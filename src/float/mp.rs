//! Multi-precision integers, and fixed point that carries its own error bound.
//!
//! The elementary functions in [`super::elementary`] cannot be computed
//! exactly — `sin 1` has no finite binary expansion — so unlike every other
//! operation in this module tree they do not hand an exact value to the
//! rounding step. They hand it an *enclosure*: an approximation together with
//! a rigorous bound on how far the true value can be from it. This file is the
//! arithmetic that keeps that bound honest.
//!
//! # The invariant
//!
//! An [`Ax`] at scale `w` stands for an unknown real number `v` with
//!
//! ```text
//! | v - (-1)^neg * mag * 2^-w |  <=  err * 2^-w
//! ```
//!
//! Every operation below takes operands satisfying that and returns a result
//! satisfying it, with `err` computed from the operands' `err` by a bound
//! derived in the operation's comment. None of it is estimated: each bound is
//! an inequality on integers, evaluated exactly, so the enclosure holds for
//! every input and every `w`. An `err` of zero therefore *means* exact, which
//! the callers rely on.
//!
//! Nothing here is floating point. A [`Big`] is a little-endian vector of
//! `u64` limbs; products go through `u128`.
//!
//! No library source of any licence was consulted; the arithmetic is the
//! schoolbook algorithms (Knuth, *TAOCP* vol. 2 §4.3.1) and the error rules
//! are first-order interval arithmetic.

use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Ordering;

/// An unsigned integer of any size: little-endian `u64` limbs, with no zero
/// limb at the top, so zero is the empty vector and equality is structural.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub(super) struct Big(Vec<u64>);

impl Big {
    /// Zero.
    pub(super) fn zero() -> Big {
        Big(Vec::new())
    }

    /// A `u64`.
    pub(super) fn from_u64(v: u64) -> Big {
        let mut b = Big(vec![v]);
        b.trim();
        b
    }

    /// A `u128`.
    #[cfg(test)]
    pub(super) fn from_u128(v: u128) -> Big {
        let mut b = Big(vec![v as u64, (v >> 64) as u64]);
        b.trim();
        b
    }

    /// `2^n`.
    pub(super) fn pow2(n: u64) -> Big {
        let mut limbs = vec![0u64; (n / 64) as usize + 1];
        limbs[(n / 64) as usize] = 1 << (n % 64);
        Big(limbs)
    }

    /// Drop zero limbs from the top.
    fn trim(&mut self) {
        while self.0.last() == Some(&0) {
            self.0.pop();
        }
    }

    /// Whether this is zero.
    pub(super) fn is_zero(&self) -> bool {
        self.0.is_empty()
    }

    /// The number of significant bits; zero for zero.
    pub(super) fn bits(&self) -> u64 {
        match self.0.last() {
            None => 0,
            Some(&top) => (self.0.len() as u64 - 1) * 64 + u64::from(64 - top.leading_zeros()),
        }
    }

    /// Bit `i`.
    pub(super) fn bit(&self, i: u64) -> bool {
        let limb = (i / 64) as usize;
        limb < self.0.len() && (self.0[limb] >> (i % 64)) & 1 != 0
    }

    /// Whether this is a power of two.
    pub(super) fn is_pow2(&self) -> bool {
        match self.0.split_last() {
            None => false,
            Some((top, rest)) => top.is_power_of_two() && rest.iter().all(|&l| l == 0),
        }
    }

    /// The low 64 bits.
    pub(super) fn low_u64(&self) -> u64 {
        self.0.first().copied().unwrap_or(0)
    }

    /// The value, if it fits in a `u128`.
    pub(super) fn to_u128(&self) -> Option<u128> {
        match self.0.len() {
            0 => Some(0),
            1 => Some(u128::from(self.0[0])),
            2 => Some(u128::from(self.0[0]) | (u128::from(self.0[1]) << 64)),
            _ => None,
        }
    }

    /// Numeric comparison.
    pub(super) fn cmp(&self, o: &Big) -> Ordering {
        match self.0.len().cmp(&o.0.len()) {
            Ordering::Equal => {}
            other => return other,
        }
        for (a, b) in self.0.iter().rev().zip(o.0.iter().rev()) {
            match a.cmp(b) {
                Ordering::Equal => {}
                other => return other,
            }
        }
        Ordering::Equal
    }

    /// `self + o`.
    pub(super) fn add(&self, o: &Big) -> Big {
        let (long, short) = if self.0.len() >= o.0.len() {
            (self, o)
        } else {
            (o, self)
        };
        let mut out = Vec::with_capacity(long.0.len() + 1);
        let mut carry = 0u64;
        for (i, &a) in long.0.iter().enumerate() {
            let b = short.0.get(i).copied().unwrap_or(0);
            let (s1, c1) = a.overflowing_add(b);
            let (s2, c2) = s1.overflowing_add(carry);
            out.push(s2);
            carry = u64::from(c1) + u64::from(c2);
        }
        if carry != 0 {
            out.push(carry);
        }
        Big(out)
    }

    /// `self + v`.
    pub(super) fn add_u64(&self, v: u64) -> Big {
        self.add(&Big::from_u64(v))
    }

    /// `self - o`, which must not be negative.
    pub(super) fn sub(&self, o: &Big) -> Big {
        debug_assert!(self.cmp(o) != Ordering::Less, "Big::sub underflow");
        let mut out = Vec::with_capacity(self.0.len());
        let mut borrow = 0u64;
        for (i, &a) in self.0.iter().enumerate() {
            let b = o.0.get(i).copied().unwrap_or(0);
            let (d1, b1) = a.overflowing_sub(b);
            let (d2, b2) = d1.overflowing_sub(borrow);
            out.push(d2);
            borrow = u64::from(b1) + u64::from(b2);
        }
        let mut r = Big(out);
        r.trim();
        r
    }

    /// `self * o`, in full.
    pub(super) fn mul(&self, o: &Big) -> Big {
        if self.is_zero() || o.is_zero() {
            return Big::zero();
        }
        let mut out = vec![0u64; self.0.len() + o.0.len()];
        for (i, &a) in self.0.iter().enumerate() {
            let mut carry = 0u128;
            for (j, &b) in o.0.iter().enumerate() {
                let t = u128::from(a) * u128::from(b) + u128::from(out[i + j]) + carry;
                out[i + j] = t as u64;
                carry = t >> 64;
            }
            out[i + o.0.len()] = carry as u64;
        }
        let mut r = Big(out);
        r.trim();
        r
    }

    /// `self * m`.
    pub(super) fn mul_u64(&self, m: u64) -> Big {
        let mut out = Vec::with_capacity(self.0.len() + 1);
        let mut carry = 0u128;
        for &a in &self.0 {
            let t = u128::from(a) * u128::from(m) + carry;
            out.push(t as u64);
            carry = t >> 64;
        }
        out.push(carry as u64);
        let mut r = Big(out);
        r.trim();
        r
    }

    /// `(floor(self / d), self mod d)`, `d` non-zero.
    pub(super) fn div_u64(&self, d: u64) -> (Big, u64) {
        debug_assert!(d != 0);
        let mut out = vec![0u64; self.0.len()];
        let mut rem = 0u128;
        for i in (0..self.0.len()).rev() {
            let cur = (rem << 64) | u128::from(self.0[i]);
            out[i] = (cur / u128::from(d)) as u64;
            rem = cur % u128::from(d);
        }
        let mut r = Big(out);
        r.trim();
        (r, rem as u64)
    }

    /// `self * 2^n`.
    pub(super) fn shl(&self, n: u64) -> Big {
        if self.is_zero() {
            return Big::zero();
        }
        let limbs = (n / 64) as usize;
        let bits = (n % 64) as u32;
        let mut out = vec![0u64; limbs];
        if bits == 0 {
            out.extend_from_slice(&self.0);
        } else {
            let mut carry = 0u64;
            for &a in &self.0 {
                out.push((a << bits) | carry);
                carry = a >> (64 - bits);
            }
            out.push(carry);
        }
        let mut r = Big(out);
        r.trim();
        r
    }

    /// `floor(self / 2^n)`, and whether anything non-zero was shifted out.
    pub(super) fn shr(&self, n: u64) -> (Big, bool) {
        let limbs = (n / 64) as usize;
        if limbs >= self.0.len() {
            return (Big::zero(), !self.is_zero());
        }
        let bits = (n % 64) as u32;
        let mut lost = self.0[..limbs].iter().any(|&l| l != 0);
        let src = &self.0[limbs..];
        let mut out = Vec::with_capacity(src.len());
        if bits == 0 {
            out.extend_from_slice(src);
        } else {
            lost |= src[0] & ((1u64 << bits) - 1) != 0;
            for i in 0..src.len() {
                let hi = src.get(i + 1).copied().unwrap_or(0);
                out.push((src[i] >> bits) | (hi << (64 - bits)));
            }
        }
        let mut r = Big(out);
        r.trim();
        (r, lost)
    }

    /// `ceil(self / 2^n)`.
    pub(super) fn shr_ceil(&self, n: u64) -> Big {
        let (q, lost) = self.shr(n);
        if lost { q.add_u64(1) } else { q }
    }

    /// `(floor(self / d), self mod d)`, `d` non-zero.
    ///
    /// Restoring binary long division: one remainder, shifted left a bit at a
    /// time, compared and subtracted in place. Quadratic, which is fine at the
    /// few hundred bits the elementary functions normally run at.
    pub(super) fn div_rem(&self, d: &Big) -> (Big, Big) {
        debug_assert!(!d.is_zero(), "Big::div_rem by zero");
        if d.0.len() == 1 {
            let (q, r) = self.div_u64(d.0[0]);
            return (q, Big::from_u64(r));
        }
        if self.cmp(d) == Ordering::Less {
            return (Big::zero(), self.clone());
        }
        let n = self.bits();
        let mut q = vec![0u64; self.0.len()];
        let mut r: Vec<u64> = vec![0u64; d.0.len() + 1];
        for i in (0..n).rev() {
            // r = 2r + bit
            let mut carry = u64::from(self.bit(i));
            for limb in r.iter_mut() {
                let next = *limb >> 63;
                *limb = (*limb << 1) | carry;
                carry = next;
            }
            if !lt_limbs(&r, &d.0) {
                sub_limbs(&mut r, &d.0);
                q[(i / 64) as usize] |= 1 << (i % 64);
            }
        }
        let mut qb = Big(q);
        qb.trim();
        let mut rb = Big(r);
        rb.trim();
        (qb, rb)
    }

    /// `ceil(self / d)`, `d` non-zero.
    pub(super) fn div_ceil(&self, d: &Big) -> Big {
        let (q, r) = self.div_rem(d);
        if r.is_zero() { q } else { q.add_u64(1) }
    }
}

/// `a < b` for limb slices that may carry zero limbs at the top.
fn lt_limbs(a: &[u64], b: &[u64]) -> bool {
    let n = a.len().max(b.len());
    for i in (0..n).rev() {
        let x = a.get(i).copied().unwrap_or(0);
        let y = b.get(i).copied().unwrap_or(0);
        if x != y {
            return x < y;
        }
    }
    false
}

/// `a -= b` in place, `a >= b`.
fn sub_limbs(a: &mut [u64], b: &[u64]) {
    let mut borrow = 0u64;
    for (i, limb) in a.iter_mut().enumerate() {
        let y = b.get(i).copied().unwrap_or(0);
        let (d1, b1) = limb.overflowing_sub(y);
        let (d2, b2) = d1.overflowing_sub(borrow);
        *limb = d2;
        borrow = u64::from(b1) + u64::from(b2);
    }
}

/// A real number known to within a bound, in fixed point at scale `2^-w`.
///
/// See the module documentation for the invariant. The scale is not stored:
/// every value in one computation shares it, and the operations that need it
/// take it as an argument.
#[derive(Debug, Clone)]
pub(super) struct Ax {
    /// The sign of the approximation.
    pub(super) neg: bool,
    /// The approximation's magnitude, in units of `2^-w`.
    pub(super) mag: Big,
    /// The error bound, in units of `2^-w`.
    pub(super) err: Big,
}

impl Ax {
    /// An exactly known value, `±mag * 2^-w`.
    pub(super) fn exact(neg: bool, mag: Big) -> Ax {
        Ax {
            neg: neg && !mag.is_zero(),
            mag,
            err: Big::zero(),
        }
    }

    /// The integer `n`, exactly.
    pub(super) fn int(n: u64, w: u32) -> Ax {
        Ax::exact(false, Big::from_u64(n).shl(u64::from(w)))
    }

    /// The same value with the sign flipped.
    pub(super) fn negated(&self) -> Ax {
        Ax {
            neg: !self.neg && !self.mag.is_zero(),
            mag: self.mag.clone(),
            err: self.err.clone(),
        }
    }

    /// An upper bound on `|v|`, in units of `2^-w`.
    pub(super) fn bound(&self) -> Big {
        self.mag.add(&self.err)
    }

    /// Widen the error bound by `e` units.
    pub(super) fn widen(&mut self, e: &Big) {
        self.err = self.err.add(e);
    }

    /// `self + o`. Exact on the approximations, so the bounds simply add.
    pub(super) fn add(&self, o: &Ax) -> Ax {
        let err = self.err.add(&o.err);
        if self.neg == o.neg {
            return Ax {
                neg: self.neg,
                mag: self.mag.add(&o.mag),
                err,
            };
        }
        match self.mag.cmp(&o.mag) {
            Ordering::Less => Ax {
                neg: o.neg,
                mag: o.mag.sub(&self.mag),
                err,
            },
            _ => {
                let mag = self.mag.sub(&o.mag);
                Ax {
                    neg: self.neg && !mag.is_zero(),
                    mag,
                    err,
                }
            }
        }
    }

    /// `self - o`.
    pub(super) fn sub(&self, o: &Ax) -> Ax {
        self.add(&o.negated())
    }

    /// `self * o`, truncated to the scale.
    ///
    /// With `a = A + da`, `b = B + db`, `|da| <= ea`, `|db| <= eb`:
    /// `ab - AB = A db + B da + da db`, so `|ab - AB| <= (|A| + ea) eb + |B| ea`.
    /// Dividing by `2^w` and truncating the product adds less than one unit,
    /// and nothing when no set bit was dropped.
    pub(super) fn mul(&self, o: &Ax, w: u32) -> Ax {
        let w = u64::from(w);
        let (mag, lost) = self.mag.mul(&o.mag).shr(w);
        let spread = self.bound().mul(&o.err).add(&o.mag.mul(&self.err));
        let mut err = spread.shr_ceil(w);
        if lost {
            err = err.add_u64(1);
        }
        Ax {
            neg: (self.neg != o.neg) && !mag.is_zero(),
            mag,
            err,
        }
    }

    /// `self * k` for an exact integer `k`: the bound scales by `k`.
    pub(super) fn mul_big(&self, k: &Big) -> Ax {
        let mag = self.mag.mul(k);
        Ax {
            neg: self.neg && !mag.is_zero(),
            mag,
            err: self.err.mul(k),
        }
    }

    /// `self * k` for an exact small integer `k`.
    pub(super) fn mul_u64(&self, k: u64) -> Ax {
        self.mul_big(&Big::from_u64(k))
    }

    /// `self / d` for an exact small integer `d`, truncated: the bound divides
    /// by `d` (rounded up), plus one unit if the quotient was inexact.
    pub(super) fn div_u64(&self, d: u64) -> Ax {
        let (mag, rem) = self.mag.div_u64(d);
        let mut err = self.err.div_ceil(&Big::from_u64(d));
        if rem != 0 {
            err = err.add_u64(1);
        }
        Ax {
            neg: self.neg && !mag.is_zero(),
            mag,
            err,
        }
    }

    /// `self / o`, truncated to the scale; `None` when the divisor's enclosure
    /// contains zero.
    ///
    /// With the notation of [`Ax::mul`]:
    /// `a/b - A/B = da/b - (A/B) db/b`, so
    /// `|a/b - A/B| <= (ea + |A/B| eb) / (|B| - eb)`. In units of `2^-w`, with
    /// `Q = A 2^w / B` the exact quotient of the approximations and
    /// `q = floor(Q)`, that is at most `(ea 2^w + (q + 1) eb) / (|B| - eb)`,
    /// rounded up, plus one unit for the truncation of `Q` to `q`.
    pub(super) fn div(&self, o: &Ax, w: u32) -> Option<Ax> {
        if o.mag.cmp(&o.err) != Ordering::Greater {
            return None;
        }
        let w64 = u64::from(w);
        let (q, r) = self.mag.shl(w64).div_rem(&o.mag);
        let numer = self.err.shl(w64).add(&q.add_u64(1).mul(&o.err));
        let mut err = numer.div_ceil(&o.mag.sub(&o.err));
        if !r.is_zero() {
            err = err.add_u64(1);
        }
        Some(Ax {
            neg: (self.neg != o.neg) && !q.is_zero(),
            mag: q,
            err,
        })
    }

    /// `self / 2^n`, truncated.
    pub(super) fn shr(&self, n: u64) -> Ax {
        let (mag, lost) = self.mag.shr(n);
        let mut err = self.err.shr_ceil(n);
        if lost {
            err = err.add_u64(1);
        }
        Ax {
            neg: self.neg && !mag.is_zero(),
            mag,
            err,
        }
    }

    /// `self * 2^n`, exactly.
    pub(super) fn shl(&self, n: u64) -> Ax {
        Ax {
            neg: self.neg,
            mag: self.mag.shl(n),
            err: self.err.shl(n),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn b(v: u128) -> Big {
        Big::from_u128(v)
    }

    #[test]
    fn big_arithmetic_agrees_with_u128() {
        let xs: [u128; 6] = [
            0,
            1,
            0xffff_ffff_ffff_ffff,
            0x1_0000_0000_0000_0000,
            0x1234_5678_9abc_def0_1357_9bdf_2468_ace0,
            u128::MAX >> 3,
        ];
        for &x in &xs {
            for &y in &xs {
                if let Some(s) = x.checked_add(y) {
                    assert_eq!(b(x).add(&b(y)).to_u128(), Some(s));
                }
                if x >= y {
                    assert_eq!(b(x).sub(&b(y)).to_u128(), Some(x - y));
                }
                if let Some(p) = x.checked_mul(y) {
                    assert_eq!(b(x).mul(&b(y)).to_u128(), Some(p));
                }
                if y != 0 {
                    let (q, r) = b(x).div_rem(&b(y));
                    assert_eq!(q.to_u128(), Some(x / y), "{x:#x} / {y:#x}");
                    assert_eq!(r.to_u128(), Some(x % y));
                }
                assert_eq!(b(x).cmp(&b(y)), x.cmp(&y));
            }
            for n in [0u64, 1, 13, 63, 64, 65, 100] {
                let (q, lost) = b(x).shr(n);
                let want = if n >= 128 { 0 } else { x >> n };
                assert_eq!(q.to_u128(), Some(want));
                assert_eq!(lost, n > 0 && x & ((1u128 << n.min(127)) - 1) != 0);
                if x.leading_zeros() as u64 >= n {
                    assert_eq!(b(x).shl(n).to_u128(), Some(x << n));
                }
            }
        }
    }

    #[test]
    fn long_division_is_exact_on_wide_operands() {
        // (2^200 + 12345) = q * d + r with d > 2^64, checked by multiplying
        // back rather than against another implementation.
        let a = Big::pow2(200).add_u64(12345);
        let d = b(0xdead_beef_cafe_babe_f00d_1234_5678);
        let (q, r) = a.div_rem(&d);
        assert_eq!(q.mul(&d).add(&r), a);
        assert_eq!(r.cmp(&d), Ordering::Less);
    }

    #[test]
    fn the_error_bounds_enclose_the_exact_results() {
        // 1/3 and 2/3 at 64 bits: the enclosures must contain the exact
        // rationals, which is checked by cross-multiplying in integers.
        let w = 64;
        let one = Ax::int(1, w);
        let third = one.div_u64(3);
        let two_thirds = Ax::int(2, w).div(&Ax::int(3, w), w).unwrap();
        let prod = third.mul(&two_thirds, w); // 2/9
        for (ax, num, den) in [(&third, 1u128, 3u128), (&two_thirds, 2, 3), (&prod, 2, 9)] {
            let lo = ax.mag.sub(&ax.err).to_u128().unwrap();
            let hi = ax.mag.add(&ax.err).to_u128().unwrap();
            // lo * den <= num * 2^w <= hi * den
            let target = num << w;
            assert!(lo * den <= target && target <= hi * den);
        }
    }
}
