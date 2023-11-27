/* Reg32 Offsets */
const VS: u32 = 0x8;
const INTMS: u32 = 0xC;
const INTMC: u32 = 0x10;
const CC: u32 = 0x14;
const CSTS: u32 = 0x1C;
const NSSR: u32 = 0x20;
const AQA: u32 = 0x24;
const CMBLOC: u32 = 0x38;
const CMBSZ: u32 = 0x3C;
const BPINFO: u32 = 0x40;
const BPRSEL: u32 = 0x44;
const BPMBL: u32 = 0x48;
const CMBSTS: u32 = 0x58;
const PMRCAP: u32 = 0xE00;
const PMRCTL: u32 = 0xE04;
const PMRSTS: u32 = 0xE08;
const PMREBS: u32 = 0xE0C;
const PMRSWTP: u32 = 0xE10;

/* Reg64 offsets */
const CAP: u32 = 0x0;
const ASQ: u32 = 0x28;
const ACQ: u32 = 0x30;
const CMBMSC: u32 = 0x50;
const PMRMSC: u32 = 0xE14;
