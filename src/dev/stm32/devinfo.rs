//! The STM32 device electronic signature: the factory-programmed words that
//! say which part this is.
//!
//! One class, `st.devinfo`. Every STM32 carries a few read-only words in its
//! system memory that firmware reads to learn about the chip it is on, and
//! ST's HAL reads them without asking:
//!
//! | Word | What it is | Who reads it |
//! | --- | --- | --- |
//! | **flash size** | the array's size in KiB, a 16-bit value | `FLASH_SIZE`, and through it every page/bank calculation in `HAL_FLASHEx_Erase` |
//! | **unique ID** | 96 bits, different on every die | `HAL_GetUIDw0/1/2`, USB serial numbers, licence keys, MAC addresses |
//! | **package** | which package the die is bonded in | `LL_GetPackageType` |
//!
//! Where they live is a fact about the *family*, so the offsets are
//! properties and the board file says where to map the block:
//!
//! | Family | Block at | `uid-offset` | `flash-size-offset` | `package-offset` | `size` |
//! | --- | --- | --- | --- | --- | --- |
//! | STM32U5 (RM0456 §75, the CMSIS `PACKAGE_BASE`/`UID_BASE`/`FLASHSIZE_BASE`) | `0x0BFA0500` | `0x200` | `0x2a0` | `0x0` | `0x2a4` |
//! | STM32F4 (RM0090 §39) | `0x1FFF7A10` | `0x0` | `0x12` | — | `0x14` |
//!
//! A U5 board that leaves this out takes a BusFault the first time the HAL
//! erases a page, because `FLASH_SIZE` is a load from `0x0BFA07A0`.
//!
//! # The unique ID is seeded, not random
//!
//! A real die's 96 bits are a wafer position and lot number. Here they come
//! from the board's `seed` mixed with the instance path, exactly as `st.rng`'s
//! numbers and `atmel.atecc`'s serial do ([`crate::core::rand`]): two runs of
//! one board see one ID, `-p seed=…` deals another, and nothing reads the host.
//!
//! # Everything else
//!
//! Reads anywhere else in the block return zero — the factory area holds more
//! (calibration values, on some parts) that nothing here models. Writes are
//! ignored, as a store into system memory is. The block has no state that
//! changes, so its snapshot chunk is empty.
//!
//! # Sources
//!
//! ST **RM0456** (STM32U5) chapter "Device electronic signature"; ST's CMSIS
//! header `stm32u585xx.h` (Apache-2.0) for `PACKAGE_BASE`, `UID_BASE` and
//! `FLASHSIZE_BASE`; ST **RM0090** §39 for the F4 offsets. No emulator source
//! of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::rand::{Stream, derive_seed};
use crate::core::space::{MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a `.machine` file uses.
pub const CLASS_NAME: &str = "st.devinfo";

/// Snapshot format version. The chunk is empty: nothing here changes.
const STATE_VERSION: u32 = 1;

/// The bytes the block answers with, laid out at their offsets.
///
/// Behind a lock only because the unique ID is filled in at realize, when the
/// instance path is known; after that nothing writes it.
struct Contents {
    bytes: Mutex<Vec<u8>>,
}

impl fmt::Debug for Contents {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Contents")
            .field("len", &self.bytes.lock().len())
            .finish_non_exhaustive()
    }
}

impl MemOps for Contents {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // Read-only and side-effect free, so a debug read is the same read.
        let start = usize::try_from(offset).map_err(|_| BusError::BadAccess)?;
        let bytes = self.bytes.lock();
        for (k, byte) in dst.iter_mut().enumerate() {
            *byte = bytes.get(start + k).copied().unwrap_or(0);
        }
        Ok(())
    }

    fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
        // System memory: a store is accepted and changes nothing.
        Ok(())
    }
}

/// The factory signature block of one STM32.
#[derive(Debug)]
pub struct Devinfo {
    contents: Arc<Contents>,
    region: RegionRef,
    /// Where the unique ID sits, for a realize that has to draw it.
    uid_offset: Option<usize>,
    /// The board's seed, mixed with the instance path at realize.
    seed: u64,
}

impl Devinfo {
    /// Build one from its properties.
    ///
    /// # Errors
    ///
    /// A field that does not fit inside `size`, or a flash size above
    /// 65535 KiB (the word is sixteen bits).
    pub fn new(props: &Props) -> Result<Devinfo> {
        let mut r = props.reader();
        let size = r.require_size("size")?;
        let size = usize::try_from(size)
            .ok()
            .filter(|s| (1..=0x1_0000).contains(s))
            .ok_or_else(|| bad("size", "between one byte and 64 KiB"))?;
        let flash_kb = r.or_range("flash-kb", 0u64, 0..=0xffff)? as u16;
        let flash_at: Option<u64> = r.optional("flash-size-offset")?;
        let uid_at: Option<u64> = r.optional("uid-offset")?;
        let package_at: Option<u64> = r.optional("package-offset")?;
        let package = r.or_range("package", 0u64, 0..=0xffff)? as u16;
        let seed: u64 = r.or("seed", 0)?;
        r.finish()?;

        let mut bytes = alloc::vec![0u8; size];
        let mut place = |name: &str, at: Option<u64>, data: &[u8]| -> Result<Option<usize>> {
            let Some(at) = at else { return Ok(None) };
            let at = usize::try_from(at).map_err(|_| bad(name, "inside the block"))?;
            let end = at.checked_add(data.len()).filter(|&e| e <= size);
            let Some(end) = end else {
                return Err(bad(name, "far enough inside `size` for the whole field"));
            };
            bytes[at..end].copy_from_slice(data);
            Ok(Some(at))
        };
        place("flash-size-offset", flash_at, &flash_kb.to_le_bytes())?;
        place("package-offset", package_at, &package.to_le_bytes())?;
        // Zero until realize draws it from the seed and the instance path.
        let uid_offset = place("uid-offset", uid_at, &[0u8; 12])?;

        let contents = Arc::new(Contents {
            bytes: Mutex::with_rank(LockRank::DEVICE, bytes),
        });
        let region = Arc::new(Region::io(
            "devinfo",
            size as u64,
            Arc::clone(&contents) as Arc<dyn MemOps>,
        ));
        Ok(Devinfo {
            contents,
            region,
            uid_offset,
            seed,
        })
    }

    /// The twelve unique-ID bytes, as the guest reads them, if the block has
    /// a unique ID at all.
    #[must_use]
    pub fn uid(&self) -> Option<[u8; 12]> {
        let at = self.uid_offset?;
        let mut out = [0u8; 12];
        out.copy_from_slice(&self.contents.bytes.lock()[at..at + 12]);
        Some(out)
    }
}

/// A property error naming the property and what it should have been.
fn bad(name: &str, wanted: &str) -> Error {
    Error::Config {
        at: String::from(name),
        message: alloc::format!("`{name}` must be {wanted}"),
    }
}

impl Device for Devinfo {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The unique ID is drawn here and not in `new`, because it mixes in
        // the instance path, which only realize knows. Nothing else is
        // outward: a `map` statement places the region.
        if let Some(at) = self.uid_offset {
            let mut uid = [0u8; 12];
            Stream::new(derive_seed(self.seed, ctx.path())).fill(&mut uid);
            self.contents.bytes.lock()[at..at + 12].copy_from_slice(&uid);
        }
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Factory-programmed: no reset of either kind changes a word of it.
    }

    fn save(&self, _w: &mut ChunkWriter<'_>) -> Result<()> {
        Ok(())
    }

    fn load(&self, _r: &mut ChunkReader<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }
}

impl Instance for Devinfo {}

/// The class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 device electronic signature: flash size, 96-bit unique ID and package, read-only",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: true,
            summary: "how many bytes the block spans (0x2a4 on a U5, 0x14 on an F4)",
        },
        PropertySpec {
            name: "flash-kb",
            kind: ValueKind::Uint,
            required: false,
            summary: "the flash size word, in KiB (2048 for a 2 MiB part)",
        },
        PropertySpec {
            name: "flash-size-offset",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the flash size word is (0x2a0 on a U5, 0x12 on an F4)",
        },
        PropertySpec {
            name: "uid-offset",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the 96-bit unique ID is (0x200 on a U5, 0x0 on an F4)",
        },
        PropertySpec {
            name: "seed",
            kind: ValueKind::Uint,
            required: false,
            summary: "the board seed the unique ID is drawn from, with the instance path",
        },
        PropertySpec {
            name: "package-offset",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the package word is (0x0 on a U5; an F4 has none)",
        },
        PropertySpec {
            name: "package",
            kind: ValueKind::Uint,
            required: false,
            summary: "the package word's value",
        },
    ],
    construct: |props| Ok(Box::new(Devinfo::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Devinfo::new(props)?)))
}

/// What the validator should know about `st.devinfo`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size).required())
        .prop(PropSchema::new("flash-kb", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("flash-size-offset", ValueKind::Uint))
        .prop(PropSchema::new("uid-offset", ValueKind::Uint))
        .prop(PropSchema::new("seed", ValueKind::Uint))
        .prop(PropSchema::new("package-offset", ValueKind::Uint))
        .prop(PropSchema::new("package", ValueKind::Uint).range(0, 0xffff))
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;

    /// The U5 layout, as `machines/stm32u575.machine` writes it.
    fn u5(seed: u64) -> Devinfo {
        let props = Props::new()
            .with("size", Value::Size(0x2a4))
            .with("flash-kb", Value::Uint(2048))
            .with("flash-size-offset", Value::Uint(0x2a0))
            .with("uid-offset", Value::Uint(0x200))
            .with("package-offset", Value::Uint(0))
            .with("package", Value::Uint(0x11))
            .with("seed", Value::Uint(seed));
        Devinfo::new(&props).expect("a valid U5 block")
    }

    fn read(d: &Devinfo, offset: u64, len: usize) -> u64 {
        let mut buf = [0u8; 8];
        d.contents
            .read(offset, &mut buf[..len], MemAttrs::DEFAULT)
            .expect("the block answers");
        u64::from_le_bytes(buf)
    }

    #[test]
    fn the_flash_size_and_package_words_are_where_the_family_puts_them() {
        let d = u5(1);
        // `FLASH_SIZE` in the HAL is a halfword load from FLASHSIZE_BASE.
        assert_eq!(read(&d, 0x2a0, 2), 2048);
        assert_eq!(read(&d, 0, 2), 0x11);
        assert_eq!(read(&d, 0x100, 4), 0, "everything else reads zero");
    }

    #[test]
    fn writes_change_nothing_and_a_field_outside_the_block_is_refused() {
        let d = u5(1);
        d.contents
            .write(0x2a0, &[0, 0], MemAttrs::DEFAULT)
            .expect("system memory takes the store");
        assert_eq!(read(&d, 0x2a0, 2), 2048);
        let props = Props::new()
            .with("size", Value::Size(0x10))
            .with("uid-offset", Value::Uint(0x8));
        assert!(Devinfo::new(&props).is_err(), "a UID that runs off the end");
    }

    #[test]
    fn the_unique_id_comes_from_the_seed_and_the_path_and_nothing_else() {
        let draw = |seed: u64, path: &str| {
            let mut uid = [0u8; 12];
            Stream::new(derive_seed(seed, path)).fill(&mut uid);
            uid
        };
        // What realize writes, without a whole machine around it.
        let d = u5(7);
        let at = d.uid_offset.expect("a U5 block has a UID");
        d.contents.bytes.lock()[at..at + 12].copy_from_slice(&draw(7, "devinfo"));
        assert_eq!(d.uid(), Some(draw(7, "devinfo")));
        assert_eq!(draw(7, "devinfo"), draw(7, "devinfo"), "two runs, one ID");
        assert_ne!(
            draw(7, "devinfo"),
            draw(8, "devinfo"),
            "another seed, another ID"
        );
        assert_ne!(draw(7, "a"), draw(7, "b"), "two instances, two IDs");
    }

    #[test]
    fn the_snapshot_round_trips_as_an_empty_chunk() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
        let saved = u5(1);
        let mut shape = MachineShape::new();
        shape.add_device("devinfo", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("devinfo", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let restored = u5(1);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("devinfo", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        // Nothing in the block changes, so the identical state is its contents.
        for offset in [0x0, 0x200, 0x204, 0x208, 0x2a0] {
            assert_eq!(read(&saved, offset, 4), read(&restored, offset, 4));
        }
    }
}
