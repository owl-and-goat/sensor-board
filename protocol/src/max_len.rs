//! The longest postcard encoding of a type, worked out at compile time from
//! the type's [`Schema`].

use postcard_schema::Schema;
use postcard_schema::schema::{DataModelType, DataModelVariant, NamedType, NamedValue};

/// The most bytes that postcard can write for a `T`. Fails to compile if `T`
/// has no upper bound, for example because it holds a sequence or a string.
///
/// The result can be larger than any real encoding, because a schema can
/// describe more than serde writes: postcard-schema gives `SocketAddrV6`
/// its flow info and scope ID, which serde leaves out.
pub const fn max_encoded_len<T: Schema>() -> usize {
    named(T::SCHEMA)
}

const fn named(schema: &NamedType) -> usize {
    match schema.ty {
        DataModelType::Bool | DataModelType::I8 | DataModelType::U8 => 1,
        DataModelType::I16 | DataModelType::U16 => varint_max(16),
        DataModelType::I32 | DataModelType::U32 => varint_max(32),
        DataModelType::I64 | DataModelType::U64 => varint_max(64),
        DataModelType::I128 | DataModelType::U128 => varint_max(128),
        // A usize is as wide as the target's pointers. Take the widest, so
        // that the host and the firmware agree.
        DataModelType::Usize | DataModelType::Isize => varint_max(64),
        DataModelType::F32 => 4,
        DataModelType::F64 => 8,
        // A char is encoded as a string: its length, then up to 4 bytes of
        // UTF-8.
        DataModelType::Char => 1 + 4,
        DataModelType::Unit | DataModelType::UnitStruct => 0,
        DataModelType::Option(inner) => 1 + named(inner),
        DataModelType::NewtypeStruct(inner) => named(inner),
        DataModelType::Tuple(fields) | DataModelType::TupleStruct(fields) => tuple(fields),
        DataModelType::Struct(fields) => values(fields),
        DataModelType::Enum(variants) => {
            let mut longest = 0;
            let mut i = 0;
            while i < variants.len() {
                let len = match variants[i].ty {
                    DataModelVariant::UnitVariant => 0,
                    DataModelVariant::NewtypeVariant(inner) => named(inner),
                    DataModelVariant::TupleVariant(fields) => tuple(fields),
                    DataModelVariant::StructVariant(fields) => values(fields),
                };
                if len > longest {
                    longest = len;
                }
                i += 1;
            }
            // The variant's index comes first, as a varint.
            varint_len(variants.len() as u64 - 1) + longest
        }
        DataModelType::String
        | DataModelType::ByteArray
        | DataModelType::Seq(_)
        | DataModelType::Map { .. }
        | DataModelType::Schema => panic!("the type has no longest encoding"),
    }
}

const fn tuple(fields: &[&NamedType]) -> usize {
    let mut len = 0;
    let mut i = 0;
    while i < fields.len() {
        len += named(fields[i]);
        i += 1;
    }
    len
}

const fn values(fields: &[&NamedValue]) -> usize {
    let mut len = 0;
    let mut i = 0;
    while i < fields.len() {
        len += named(fields[i].ty);
        i += 1;
    }
    len
}

/// The longest varint of an integer that is `bits` wide. Each byte holds 7
/// bits.
const fn varint_max(bits: usize) -> usize {
    bits.div_ceil(7)
}

/// The length of the varint that encodes `value`.
const fn varint_len(value: u64) -> usize {
    let bits = (u64::BITS - value.leading_zeros()) as usize;
    if bits == 0 { 1 } else { varint_max(bits) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::net::SocketAddrV6;
    use core::time::Duration;

    #[test]
    fn primitives() {
        assert_eq!(max_encoded_len::<u8>(), 1);
        assert_eq!(max_encoded_len::<u16>(), 3);
        assert_eq!(max_encoded_len::<u32>(), 5);
        assert_eq!(max_encoded_len::<u64>(), 10);
        assert_eq!(max_encoded_len::<Duration>(), 10 + 5);
        assert_eq!(max_encoded_len::<Option<[u8; 16]>>(), 1 + 16);
    }

    #[test]
    fn socket_address_includes_what_serde_leaves_out() {
        assert_eq!(max_encoded_len::<SocketAddrV6>(), 16 + 3 + 5 + 5);
    }

    #[test]
    fn enum_index_is_a_varint() {
        assert_eq!(varint_len(0), 1);
        assert_eq!(varint_len(127), 1);
        assert_eq!(varint_len(128), 2);
        assert_eq!(max_encoded_len::<Result<u8, u32>>(), 1 + 5);
    }
}
