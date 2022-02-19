use fixed::types::I23F9;
use fixed_macro::fixed;

// const K_TYPE_T0: [I23F9; 10] = [
//     fixed!(0.0000000E+00: I23F9),
//     2.5173462E+01,
//     -1.1662878E+00,
//     -1.0833638E+00,
//     -8.9773540E-01,
//     -3.7342377E-01,
//     -8.6632643E-02,
//     -1.0450598E-02,
//     -5.1920577E-04,
//     0.0000000E+00,
// ];

const K_TYPE_T0: [I23F9; 10] = [
    fixed!(0.0000000E+00: I23F9),
    fixed!(2.5173462E+01: I23F9),
    fixed!(-1.1662878E+00: I23F9),
    fixed!(-1.0833638E+00: I23F9),
    fixed!(-8.9773540E-01: I23F9),
    fixed!(-3.7342377E-01: I23F9),
    fixed!(-8.6632643E-02: I23F9),
    fixed!(-1.0450598E-02: I23F9),
    fixed!(-5.1920577E-04: I23F9),
    fixed!(0.0000000E+00: I23F9),
];

// const K_TYPE_T1: [FP; 10] = [
//     0.000000E+00,
//     2.508355E+01,
//     7.860106E-02,
//     -2.503131E-01,
//     8.315270E-02,
//     -1.228034E-02,
//     9.804036E-04,
//     -4.413030E-05,
//     1.057734E-06,
//     -1.052755E-08,
// ];

const K_TYPE_T1: [I23F9; 10] = [
    fixed!(0.0000000E+00: I23F9),
    fixed!(2.508355E+01: I23F9),
    fixed!(7.860106E-02: I23F9),
    fixed!(-2.503131E-01: I23F9),
    fixed!(8.315270E-02: I23F9),
    fixed!(-1.228034E-02: I23F9),
    fixed!(9.804036E-04: I23F9),
    fixed!(-4.413030E-05: I23F9),
    fixed!(1.057734E-06: I23F9),
    fixed!(-1.052755E-08: I23F9),
];

// const K_TYPE_T2: [FP; 10] = [
//     -1.318058E+02,
//     4.830222E+01,
//     -1.646031E+00,
//     5.464731E-02,
//     -9.650715E-04,
//     8.802193E-06,
//     -3.110810E-08,
//     0.000000E+00,
//     0.000000E+00,
//     0.000000E+00,
// ];

const K_TYPE_T2: [I23F9; 10] = [
    fixed!(-1.318058E+02: I23F9),
    fixed!(4.830222E+01: I23F9),
    fixed!(-1.646031E+00: I23F9),
    fixed!(5.464731E-02: I23F9),
    fixed!(-9.650715E-04: I23F9),
    fixed!(8.802193E-06: I23F9),
    fixed!(-3.110810E-08: I23F9),
    fixed!(0.000000E+00: I23F9),
    fixed!(0.000000E+00: I23F9),
    fixed!(0.000000E+00: I23F9),
];


// pub fn t(e: Millivolts) -> Celsius {
//     let e = e.0;
//     #[cfg(all(feature = "f32", not(feature = "extrapolate")))]
//     const TOL: FP = 0.005; // Tolerance for E(T) range
//     #[cfg(all(feature = "f64", not(feature = "extrapolate")))]
//     const TOL: FP = 0.0005; // Tolerance for E(T) range

//     #[cfg(not(any(feature = "extrapolate")))]
//     assert!(e >= -5.891 - TOL);
//     #[cfg(not(any(feature = "extrapolate")))]
//     assert!(e <= 54.886 + TOL);

//     let c = match (e < 0.0, e < 20.644) {
//         (true, _) => K_TYPE_T0,
//         (false, true) => K_TYPE_T1,
//         (false, false) => K_TYPE_T2,
//     };

//     // Power Series
//     let ps = c[0]
//         + c[1] * e
//         + c[2] * e * e
//         + c[3] * e * e * e
//         + c[4] * e * e * e * e
//         + c[5] * e * e * e * e * e
//         + c[6] * e * e * e * e * e * e
//         + c[7] * e * e * e * e * e * e * e
//         + c[8] * e * e * e * e * e * e * e * e
//         + c[9] * e * e * e * e * e * e * e * e * e;

//     Celsius(ps)
// }

pub fn t(millivolts: I23F9) -> i16 {
    const TOL: I23F9 = fixed!(0.005: I23F9); // Tolerance for E(T) range

    let e = millivolts;

    assert!(e >= fixed!(-5.891: I23F9) - TOL);
    assert!(e <= fixed!(54.886: I23F9) + TOL);

    let c = match (e < fixed!(0.0: I23F9), e < fixed!(20.644: I23F9)) {
        (true, _) => &K_TYPE_T0,
        (false, true) => &K_TYPE_T1,
        (false, false) => &K_TYPE_T2,
    };

    // Power Series
    let ps = c[0]
        + c[1] * e
        + c[2] * e * e
        + c[3] * e * e * e
        + c[4] * e * e * e * e
        + c[5] * e * e * e * e * e
        + c[6] * e * e * e * e * e * e
        + c[7] * e * e * e * e * e * e * e
        + c[8] * e * e * e * e * e * e * e * e
        + c[9] * e * e * e * e * e * e * e * e * e;
    
    ps.to_num()
}
