//! What the host has asked to see: the exit mask, the `INT n` vectors that
//! count as system calls, and the host-armed execution breakpoints.
//!
//! The level-3 seam (`core::exec`) on this core. The rules every reason
//! follows are in `docs/cpu/x86.md` ("Exits to the host"); this file holds
//! the configuration and the two small public types an x86 consumer needs to
//! read an [`Exit`](crate::core::exec::Exit) — [`VectorSet`] going in and
//! [`FaultDetail`] coming out.
//!
//! **Configuration, not architectural state.** None of it is in the snapshot
//! and a reset does not clear it, for the reason `ExitingCore::set_exit_mask`
//! gives: the consumer that armed it is the one still there after a restore.

use alloc::vec::Vec;
use core::ops::Range;

use crate::core::exec::{ExitMask, ExitReason};

/// A set of the 256 interrupt vectors — which `INT n` instructions leave the
/// core as [`ExitReason::SYSCALL`] rather than going through the guest's
/// interrupt table.
///
/// A host-selected set rather than "every `INT n`" because a user-mode guest
/// uses both kinds in one program: a Win16 task calls `INT 21h` for the
/// operating system the host is standing in for, and `INT 10h` for a BIOS the
/// host may well have left to guest code. Linux's `INT 80h`, DOS's `INT 21h`
/// and NT's `INT 2Eh` are the usual members.
///
/// `Copy`, thirty-two bytes, `const`-constructible, so a consumer can name its
/// personality's set as a constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct VectorSet([u64; 4]);

impl VectorSet {
    /// No vector: every `INT n` goes through the interrupt table.
    pub const NONE: VectorSet = VectorSet([0; 4]);

    /// Every vector.
    pub const ALL: VectorSet = VectorSet([u64::MAX; 4]);

    /// The same set with `vector` added.
    #[must_use]
    pub const fn with(self, vector: u8) -> VectorSet {
        let mut bits = self.0;
        bits[(vector >> 6) as usize] |= 1 << (vector & 63);
        VectorSet(bits)
    }

    /// The same set with `vector` removed.
    #[must_use]
    pub const fn without(self, vector: u8) -> VectorSet {
        let mut bits = self.0;
        bits[(vector >> 6) as usize] &= !(1 << (vector & 63));
        VectorSet(bits)
    }

    /// Whether `vector` is in the set.
    #[inline]
    #[must_use]
    pub const fn contains(self, vector: u8) -> bool {
        self.0[(vector >> 6) as usize] & (1 << (vector & 63)) != 0
    }

    /// Whether the set is empty.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0[0] | self.0[1] | self.0[2] | self.0[3] == 0
    }

    /// The raw bits, vector `n` at bit `n % 64` of word `n / 64`.
    #[must_use]
    pub const fn bits(self) -> [u64; 4] {
        self.0
    }

    /// A set from raw bits, laid out as [`bits`](VectorSet::bits) returns them.
    #[must_use]
    pub const fn from_bits(bits: [u64; 4]) -> VectorSet {
        VectorSet(bits)
    }
}

/// The x86 reading of [`Exit::detail`](crate::core::exec::Exit::detail) for an
/// [`ExitReason::FAULT`]: the exception vector and, where the vector pushes
/// one, its error code.
///
/// Both travel in the one `u64` the architecture-neutral [`Exit`] has room
/// for, the way the AArch64 core packs its exception class and syndrome:
///
/// | Bits | Holds |
/// | --- | --- |
/// | 0-7 | the vector (`6` for `#UD`, `13` for `#GP`, `14` for `#PF`) |
/// | 8 | set when an error code is present |
/// | 32-63 | the error code |
///
/// So `detail & 0xff` is the vector on every exit, and a consumer that only
/// wants to know *which* exception never needs this type. The error code is
/// an [`Option`] for the reason `exec::Fault`'s is: whether a vector has one
/// at all is part of its definition, and `#GP(0)` is not "no error code".
///
/// [`Exit`]: crate::core::exec::Exit
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultDetail {
    /// The exception vector.
    pub vector: u8,
    /// The error code the vector pushes, where it pushes one.
    pub error: Option<u32>,
}

impl FaultDetail {
    /// Bit 8: an error code is present.
    const ERROR_VALID: u64 = 1 << 8;

    /// Pack into an [`Exit::detail`](crate::core::exec::Exit::detail).
    #[must_use]
    pub const fn to_detail(self) -> u64 {
        let vector = self.vector as u64;
        match self.error {
            Some(error) => vector | Self::ERROR_VALID | ((error as u64) << 32),
            None => vector,
        }
    }

    /// Unpack an [`Exit::detail`](crate::core::exec::Exit::detail).
    #[must_use]
    pub const fn from_detail(detail: u64) -> FaultDetail {
        FaultDetail {
            vector: detail as u8,
            error: if detail & Self::ERROR_VALID != 0 {
                Some((detail >> 32) as u32)
            } else {
                None
            },
        }
    }
}

/// Everything the host has armed on one core.
///
/// Lives in the core's session, beside the state it is consulted against,
/// so reading it costs no lock the step does not already hold.
#[derive(Debug, Clone, Default)]
pub(super) struct HostExits {
    /// Which reasons leave the core.
    pub(super) mask: ExitMask,
    /// Which `INT n` vectors count as [`ExitReason::SYSCALL`].
    pub(super) vectors: VectorSet,
    /// The armed execution breakpoints as **inclusive** `(first, last)`
    /// linear-address pairs, sorted by `first`, duplicates allowed.
    ///
    /// Inclusive so that a breakpoint on the very last address of the space
    /// is expressible, and a list rather than a merged set so that removing
    /// one of two overlapping ranges leaves the other exactly as it was
    /// armed.
    breaks: Vec<(u64, u64)>,
}

impl HostExits {
    /// Whether anything at all is armed — the one test an `Exec` is built
    /// against, so a core nobody asked anything of carries a `None` and pays
    /// one predictable branch per instruction for all of this.
    #[inline]
    pub(super) fn active(&self) -> bool {
        !self.mask.is_empty() || !self.breaks.is_empty()
    }

    /// Whether `reason` leaves the core.
    #[inline]
    pub(super) fn exits(&self, reason: ExitReason) -> bool {
        self.mask.contains(reason)
    }

    /// Whether any breakpoint is armed.
    #[inline]
    pub(super) fn has_breaks(&self) -> bool {
        !self.breaks.is_empty()
    }

    /// Whether an instruction starting at `linear` stops.
    ///
    /// A binary search for the candidates, then a scan of them: entries are
    /// sorted by their first address, so only those starting at or below
    /// `linear` can hold it.
    #[inline]
    pub(super) fn breaks_at(&self, linear: u64) -> bool {
        let candidates = self.breaks.partition_point(|&(first, _)| first <= linear);
        self.breaks[..candidates]
            .iter()
            .any(|&(_, last)| linear <= last)
    }

    /// Arm the inclusive pair `(first, last)`.
    pub(super) fn add(&mut self, pair: (u64, u64)) {
        let at = self.breaks.partition_point(|&p| p <= pair);
        self.breaks.insert(at, pair);
    }

    /// Disarm one breakpoint armed as exactly `(first, last)`.
    pub(super) fn remove(&mut self, pair: (u64, u64)) -> bool {
        match self.breaks.iter().position(|&p| p == pair) {
            Some(at) => {
                self.breaks.remove(at);
                true
            }
            None => false,
        }
    }

    /// Disarm every breakpoint.
    pub(super) fn clear(&mut self) {
        self.breaks.clear();
    }

    /// The armed breakpoints, as half-open ranges where they fit one.
    pub(super) fn list(&self) -> Vec<Range<u64>> {
        self.breaks
            .iter()
            .map(|&(first, last)| first..last.saturating_add(1))
            .collect()
    }

    /// A half-open range as the inclusive pair this list holds, or `None`
    /// for an empty one.
    pub(super) fn pair(range: Range<u64>) -> Option<(u64, u64)> {
        (range.start < range.end).then(|| (range.start, range.end - 1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_vector_set_holds_what_was_put_in_it() {
        let set = VectorSet::NONE.with(0x21).with(0x80).with(0xff).with(0);
        for v in [0u8, 0x21, 0x80, 0xff] {
            assert!(set.contains(v), "vector {v:#x}");
        }
        assert!(!set.contains(0x2e));
        assert!(!set.without(0x80).contains(0x80));
        assert!(VectorSet::NONE.is_empty());
        assert!(!set.is_empty());
        assert_eq!(VectorSet::from_bits(set.bits()), set);
        assert!(VectorSet::ALL.contains(0x2e));
    }

    #[test]
    fn a_fault_detail_round_trips_with_and_without_an_error_code() {
        for detail in [
            FaultDetail {
                vector: 6,
                error: None,
            },
            FaultDetail {
                vector: 13,
                error: Some(0),
            },
            FaultDetail {
                vector: 14,
                error: Some(0xffff_ffff),
            },
        ] {
            let packed = detail.to_detail();
            assert_eq!(packed & 0xff, u64::from(detail.vector));
            assert_eq!(FaultDetail::from_detail(packed), detail);
        }
        // `#UD` packs to its bare vector, so a consumer comparing against the
        // number gets the answer it expects.
        assert_eq!(
            FaultDetail {
                vector: 6,
                error: None
            }
            .to_detail(),
            6
        );
    }

    #[test]
    fn breakpoints_are_inclusive_overlapping_and_removed_one_at_a_time() {
        let mut host = HostExits::default();
        assert!(!host.active());
        host.add((0x1000, 0x1000));
        host.add(HostExits::pair(0xfffe_0000..0xffff_0000).unwrap());
        host.add((0xfffe_8000, 0xfffe_8000));
        host.add((u64::MAX, u64::MAX));
        assert_eq!(HostExits::pair(5..5), None, "an empty range arms nothing");
        assert!(host.active());
        assert!(host.breaks_at(0x1000));
        assert!(!host.breaks_at(0x1001));
        assert!(!host.breaks_at(0xfff));
        assert!(host.breaks_at(0xfffe_0010));
        assert!(host.breaks_at(0xfffe_ffff));
        assert!(!host.breaks_at(0xffff_0000));
        assert!(host.breaks_at(u64::MAX));
        assert!(!host.breaks_at(u64::MAX - 1));

        // Removing the window leaves the single address inside it armed.
        let window = HostExits::pair(0xfffe_0000..0xffff_0000).unwrap();
        assert!(host.remove(window));
        assert!(!host.breaks_at(0xfffe_0010));
        assert!(host.breaks_at(0xfffe_8000));
        assert!(!host.remove(window), "already gone");
        assert_eq!(host.list().len(), 3);
        host.clear();
        assert!(!host.has_breaks());
    }
}
