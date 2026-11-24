//! Declares registry-backed name sets: one variant per listed value, and an
//! `Other` arm that keeps any other spelling verbatim.
//!
//! Matching is exact. Aliases (`ITRF-97` and `ITRF1997`) stay distinct, so a
//! message writes back with the spelling it was read with.

macro_rules! open_enum {
    (
        $(#[$meta:meta])*
        $name:ident { $($(#[$vmeta:meta])* $variant:ident => $text:literal,)+ }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum $name {
            $($(#[$vmeta])* $variant,)+
            /// A value outside the listed set, kept as written.
            Other(String),
        }

        impl $name {
            /// Maps a name to its variant; unlisted names become `Other`.
            pub fn from_name(name: &str) -> Self {
                match name {
                    $($text => Self::$variant,)+
                    other => Self::Other(other.to_owned()),
                }
            }

            /// The name as written in a message.
            pub fn as_str(&self) -> &str {
                match self {
                    $(Self::$variant => $text,)+
                    Self::Other(other) => other,
                }
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(self.as_str())
            }
        }
    };
}

pub(crate) use open_enum;
