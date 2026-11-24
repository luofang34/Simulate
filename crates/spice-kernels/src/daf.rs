//! The Double Precision Array File container that every binary SPICE kernel
//! is stored in.

mod decode;
mod encode;
mod layout;
#[cfg(test)]
mod tests;

use alloc::string::String;
use alloc::vec::Vec;

use crate::error::Error;

/// Checks a fixed-width text field: printable ASCII, at most `max` characters.
pub(crate) fn check_text(field: &'static str, text: &str, max: usize) -> Result<(), Error> {
    layout::check_printable(field, text)?;
    if text.len() > max {
        return Err(Error::TextTooLong {
            field,
            len: text.len(),
            max,
        });
    }
    Ok(())
}

/// IEEE byte order of a DAF, named by its binary format word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Endian {
    /// `LTL-IEEE`.
    Little,
    /// `BIG-IEEE`.
    Big,
}

/// One DAF array with its summary and name.
#[derive(Clone, Debug, PartialEq)]
pub struct DafArray {
    name: String,
    doubles: Vec<f64>,
    integers: Vec<i32>,
    data: Vec<f64>,
}

impl DafArray {
    /// Builds an array from its name, the `ND` double and `NI - 2` integer
    /// summary components, and its data words. The two address components
    /// that end every summary are assigned when the file is encoded.
    pub fn new(
        name: &str,
        doubles: Vec<f64>,
        integers: Vec<i32>,
        data: Vec<f64>,
    ) -> Result<Self, Error> {
        layout::check_printable("array name", name)?;
        if data.is_empty() {
            return Err(Error::EmptyArray { name: name.into() });
        }
        Ok(Self {
            name: name.into(),
            doubles,
            integers,
            data,
        })
    }

    /// Array name, trailing padding removed.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Double precision summary components.
    #[must_use]
    pub fn doubles(&self) -> &[f64] {
        &self.doubles
    }

    /// Integer summary components without the two trailing addresses.
    #[must_use]
    pub fn integers(&self) -> &[i32] {
        &self.integers
    }

    /// The array's data words.
    #[must_use]
    pub fn data(&self) -> &[f64] {
        &self.data
    }
}

/// An in-memory DAF: what [`Daf::parse`] reads and [`Daf::to_bytes`] writes.
#[derive(Clone, Debug, PartialEq)]
pub struct Daf {
    kernel_type: String,
    nd: usize,
    ni: usize,
    internal_name: String,
    comment_lines: Vec<String>,
    spare_comment_records: usize,
    arrays: Vec<DafArray>,
}

impl Daf {
    /// Starts an empty DAF.
    ///
    /// `kernel_type` is what follows `DAF/` in the identification word (`SPK`,
    /// `CK`, `PCK`); `nd` and `ni` are the double and integer component counts
    /// of every summary, `ni` including the two addresses; `internal_name` is
    /// the internal file name (`LOCIFN`), at most 60 printable characters.
    pub fn new(
        kernel_type: &str,
        nd: usize,
        ni: usize,
        internal_name: &str,
    ) -> Result<Self, Error> {
        check_text("kernel type", kernel_type, layout::KERNEL_TYPE_CHARS)?;
        layout::summary_words(nd, ni)?;
        check_text(
            "internal file name",
            internal_name,
            layout::INTERNAL_NAME_CHARS,
        )?;
        Ok(Self {
            kernel_type: kernel_type.into(),
            nd,
            ni,
            internal_name: internal_name.into(),
            comment_lines: Vec::new(),
            spare_comment_records: 0,
            arrays: Vec::new(),
        })
    }

    /// Parses a DAF of either IEEE byte order.
    pub fn parse(bytes: &[u8]) -> Result<Self, Error> {
        decode::parse(bytes)
    }

    /// Encodes the file, little-endian.
    pub fn to_bytes(&self) -> Result<Vec<u8>, Error> {
        encode::encode(self, Endian::Little)
    }

    /// Kernel type from the identification word, such as `SPK`.
    #[must_use]
    pub fn kernel_type(&self) -> &str {
        &self.kernel_type
    }

    /// Double precision components per summary (`ND`).
    #[must_use]
    pub const fn nd(&self) -> usize {
        self.nd
    }

    /// Integer components per summary (`NI`), the two addresses included.
    #[must_use]
    pub const fn ni(&self) -> usize {
        self.ni
    }

    /// Internal file name (`LOCIFN`).
    #[must_use]
    pub fn internal_name(&self) -> &str {
        &self.internal_name
    }

    /// Comment area text, one entry per line.
    #[must_use]
    pub fn comment_lines(&self) -> &[String] {
        &self.comment_lines
    }

    /// Replaces the comment area text. A line may hold any character up to
    /// U+00FF except NUL and EOT, which delimit the comment area; SPICE
    /// tools expect printable ASCII.
    pub fn set_comment_lines(&mut self, lines: Vec<String>) -> Result<(), Error> {
        for line in &lines {
            layout::check_comment_line(line)?;
        }
        self.comment_lines = lines;
        Ok(())
    }

    /// Empty comment records the file holds after its comment text, room for
    /// SPICE tools to add comments in place.
    #[must_use]
    pub const fn spare_comment_records(&self) -> usize {
        self.spare_comment_records
    }

    /// Sets the number of empty comment records written after the comment
    /// text.
    pub const fn set_spare_comment_records(&mut self, records: usize) {
        self.spare_comment_records = records;
    }

    /// Arrays in file order.
    #[must_use]
    pub fn arrays(&self) -> &[DafArray] {
        &self.arrays
    }

    /// Appends an array after checking that its summary fits this file.
    pub fn push(&mut self, array: DafArray) -> Result<(), Error> {
        let free_integers = self.ni.saturating_sub(2);
        if array.doubles.len() != self.nd || array.integers.len() != free_integers {
            return Err(Error::SummaryShapeMismatch {
                name: array.name,
                doubles: array.doubles.len(),
                integers: array.integers.len(),
                nd: self.nd,
                ni: free_integers,
            });
        }
        let name_chars = layout::summary_words(self.nd, self.ni)?.saturating_mul(8);
        if array.name.len() > name_chars {
            return Err(Error::TextTooLong {
                field: "array name",
                len: array.name.len(),
                max: name_chars,
            });
        }
        self.arrays.push(array);
        Ok(())
    }
}
