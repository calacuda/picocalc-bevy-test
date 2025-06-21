pub enum KeyState {
    KeyStateIdle,
    KeyStatePressed,
    KeyStateHold,
    KeyStateReleased,
}

pub const KEY_JOY_UP: u8 = 0x0;
pub const KEY_JOY_DOWN: u8 = 0x02;
pub const KEY_JOY_LEFT: u8 = 0x03;
pub const KEY_JOY_RIGHT: u8 = 0x04;
pub const KEY_JOY_CENTER: u8 = 0x05;
pub const KEY_BTN_LEFT1: u8 = 0x06;
pub const KEY_BTN_RIGHT1: u8 = 0x07;

pub const KEY_BACKSPACE: u8 = 0x08;
pub const KEY_TAB: u8 = 0x09;
pub const KEY_ENTER: u8 = 0x0A;
// 0x0D - CARRIAGE RETURN
pub const KEY_BTN_LEFT2: u8 = 0x11;
pub const KEY_BTN_RIGHT2: u8 = 0x12;

pub const KEY_MOD_ALT: u8 = 0xA1;
pub const KEY_MOD_SHL: u8 = 0xA2;
pub const KEY_MOD_SHR: u8 = 0xA3;
pub const KEY_MOD_SYM: u8 = 0xA4;
pub const KEY_MOD_CTRL: u8 = 0xA5;

pub const KEY_ESC: u8 = 0xB1;
pub const KEY_UP: u8 = 0xb5;
pub const KEY_DOWN: u8 = 0xb6;
pub const KEY_LEFT: u8 = 0xb4;
pub const KEY_RIGHT: u8 = 0xb7;

pub const KEY_BREAK: u8 = 0xd0; // == KEY_PAUSE
pub const KEY_INSERT: u8 = 0xD1;
pub const KEY_HOME: u8 = 0xD2;
pub const KEY_DEL: u8 = 0xD4;
pub const KEY_END: u8 = 0xD5;
pub const KEY_PAGE_UP: u8 = 0xd6;
pub const KEY_PAGE_DOWN: u8 = 0xd7;

pub const KEY_CAPS_LOCK: u8 = 0xC1;

pub const KEY_F1: u8 = 0x81;
pub const KEY_F2: u8 = 0x82;
pub const KEY_F3: u8 = 0x83;
pub const KEY_F4: u8 = 0x84;
pub const KEY_F5: u8 = 0x85;
pub const KEY_F6: u8 = 0x86;
pub const KEY_F7: u8 = 0x87;
pub const KEY_F8: u8 = 0x88;
pub const KEY_F9: u8 = 0x89;
pub const KEY_F10: u8 = 0x90;
