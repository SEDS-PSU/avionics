#![no_std]
#![cfg_attr(test, no_main)]

use output_board as _; // memory layout + panic handler

#[defmt_test::tests]
mod tests {}
