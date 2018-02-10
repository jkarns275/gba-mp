#![feature(i128_type, asm, proc_macro, lang_items, core, core_intrinsics, const_fn, untagged_unions, arbitrary_self_types)]
#![no_std]

#![allow(dead_code)]

extern crate gbalib;
use gbalib::alloc::*;
use gbalib::boxed::Ptr;
use gbalib::mem::{ memcpy };
use gbalib::collections::*;
use gbalib::graphics::sprites::*;
use gbalib::graphics::*;

//#[macro_use]
extern crate gbaimg;
use gbaimg::*;

use core::intrinsics::*;
use core::*;

#[repr(u8)]
enum BitState {
    High = 1,
    Low = 0
}

use BitState::*;

const GBA_ID: Ptr<u16> = Ptr::from_u32(0x03000000);
const REG_IME: Ptr<u16> = Ptr::from_u32(0x04000208);
const REG_IE: Ptr<u16> = Ptr::from_u32(0x04000200);
const REG_VCOUNT: Ptr<u16> = Ptr::from_u32(0x04000006);
const REG_KEYINPUT: Ptr<u16> = Ptr::from_u32(0x04000130);
const REG_RCNT: Ptr<u16> = Ptr::from_u32(0x04000134);
const REG_SIOCNT: Ptr<u16> = Ptr::from_u32(0x04000128);
const REG_DATA: Ptr<u16> = Ptr::from_u32(0x0400012A);
const DATA_0: Ptr<u16> = Ptr::from_u32(0x04000120);
const DATA_1: Ptr<u16> = Ptr::from_u32(0x04000122);
const DATA_2: Ptr<u16> = Ptr::from_u32(0x04000124);
const DATA_3: Ptr<u16> = Ptr::from_u32(0x04000126);
const VRAM: Ptr<u16> = Ptr::from_u32(0x06000000);

const DEBUG_REG: Ptr<u16> = Ptr::from_u32(0x03000000);

use core::any::Any;

#[inline]
fn get_bit_n(n: u16, value: u16) -> u16 {
    ((value >> n) & 1).into()
}

unsafe fn get_mp_id() -> u16 {
    (volatile_load(REG_SIOCNT.ptr) & 0x30) >> 4
}

unsafe fn set_bit_n(value: BitState, n: u32, ptr: Ptr<u16>) {
    let p = volatile_load(ptr.transmute::<u32>().ptr) & !(1 << n);
    let mut new_ptr = ptr.transmute::<u32>();
    volatile_store(new_ptr.ptr_mut, p | ((value as u32) << n));
}

fn left_pressed(keyinput: u16) -> bool  { (!keyinput & 0b100000) != 0 }
fn right_pressed(keyinput: u16) -> bool { (!keyinput & 0b10000) != 0 }
fn up_pressed(keyinput: u16) -> bool    { (!keyinput & 0b1000000) != 0 }
fn down_pressed(keyinput: u16) -> bool  { (!keyinput & 0b10000000) != 0 }
fn b_pressed(keyinput: u16) -> bool     { (!keyinput & 2) != 0 }
fn a_pressed(keyinput: u16) -> bool     { (!keyinput & 1) != 0 }

unsafe fn data_transfer(data: u16) -> [u16; 4] {
    // Configure REG_SIOCNT for multiplayer mode
    set_bit_n(Low, 12, REG_SIOCNT);
    set_bit_n(High, 13, REG_SIOCNT);

    // Configure REG_RCNT for multiplayer mode
    set_bit_n(Low, 15, REG_RCNT);


    // Wait to verify that all GBAs are in multiplayer mode
    while get_bit_n(3, *REG_SIOCNT) != 1 {}

    let mut master = false;

    // If the 2nd bit of REG_SIOCNT is 0, then this is the master GBA
    if get_bit_n(2, *REG_SIOCNT) == 0 {
        master = true;

        volatile_store(REG_DATA.ptr_mut, data);

        // Set the Start/Busy bit (7th bit), this also sets SC to Low which starts
        // the data transfer protocol.
        set_bit_n(High, 7, REG_SIOCNT);
        while get_mp_id() != 0 {}
        *GBA_ID = get_mp_id();
        let mut i = 0;

        // Wait until the busy bit isn't sent - data transfer protocol is over.
        while get_bit_n(7, volatile_load(REG_SIOCNT.ptr)) != 0 {}

    } else { // Otherwise, this is a child GBA else
        *REG_DATA = data;
        let mut i = 0;

        *GBA_ID = get_mp_id();

        // Wait until the busy bit isn't sent - data transfer protocol is over.
        while get_bit_n(7, volatile_load(REG_SIOCNT.ptr)) != 0 {}
    }


    // Configure REG_SIOCNT to NOT be in multiplayer mode.
    set_bit_n(High, 12, REG_SIOCNT);
    set_bit_n(Low, 13, REG_SIOCNT);

    // Configure REG_RCNT to NOT be in multiplayer mode
    set_bit_n(High, 15, REG_RCNT);

    volatile_load(DATA_0.transmute().ptr)
}

macro_rules! cpu_interrupt {
    ($int: expr) => (
        asm!(concat!("swi ", $int) : : : "r0", "r1", "r2", "r3" :);
    )
}

unsafe fn vsync() {
    // TODO: Make this work
    //cpu_interrupt!(0x05);
    while volatile_load(REG_VCOUNT.ptr) >= 160 {}
    while volatile_load(REG_VCOUNT.ptr) < 160 {}
}

unsafe fn clear_screen() {
    let mut x = 0;
    let mut y = 0;
    while x < 240 {
        y = 0;
        while y < 160 {
            let mut p =
                VRAM
                .clone()
                .offset((y * 240 + x) as i32);
            volatile_store(p.ptr_mut, 0);
            y += 1;
        }
        x += 1;
    }
}

#[inline]
unsafe fn plot_pixel(x: u32, y: u32, clr: u16) {
    let mut p =
        VRAM
        .clone()
        .offset((y * 240 + x) as i32);
    volatile_store(p.ptr_mut, clr);
}

#[no_mangle]
pub unsafe extern "C" fn main(_: i32, _: *const *const i8) -> i32 {
    /*
     * A simple test for sending data using the multiplayer data transfer mode as per:
     * http://problemkaputt.de/gbatek.htm#siomultiplayermode
     *
     */
    let mut graphics_mode = GraphicsMode::default();
    graphics_mode.vm = VideoMode::Mode3;
    graphics_mode.bg2_enabled = true;
    graphics_mode.set();

    //clear_screen();

    let (mut x, mut y) = ([120u32, 120, 120, 120], [80u32, 80, 80, 80]);
    let colors = [0xFFFFu16, 0xFF00, 0x01E0, 0x000F];
    loop {
        vsync();
        let this_input = volatile_load(REG_KEYINPUT.ptr);
        let all_inputs: [u16; 4] = data_transfer(this_input);
        //let other_input = input;

        for i in 0..4usize {
            if i as u16 == *GBA_ID { continue }
            let input = all_inputs[i];
            if input & 0x03FF == 0x03FF {
                continue
            }

            if b_pressed(input) {
                clear_screen();
            }
            if up_pressed(input) {
                if y[i] > 0 { y[i] -= 1; } else { y[i] = 159; }
            }
            if down_pressed(input) {
                if y[i] < 159 { y[i] += 1; } else { y[i] = 0; }
            }
            if left_pressed(input) {
                if x[i] > 0 { x[i] -= 1; } else { x[i] = 239; }
            }
            if right_pressed(input) {
                if x[i] < 239 { x[i] += 1; } else { x[i] = 0; }
            }
            plot_pixel(x[i], y[i], colors[i]);
        }
    }

}
