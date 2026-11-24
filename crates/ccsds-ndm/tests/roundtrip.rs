//! Reads the published example messages and real producer samples, writes
//! them back, and checks that nothing is lost.

#[cfg(test)]
mod roundtrip {
    pub mod content;
    pub mod ndm;
    pub mod oem;
    pub mod omm;
    pub mod opm;
}
