pub mod fixture_ {
    #[derive(Debug, Default, PartialEq, Clone)]
    pub struct DeviceInfo {
        pub r#fw_version: ::heapless::String<64>,
        pub r#serial: u32,
    }
    impl DeviceInfo {
        /// Return a reference to `fw_version`
        #[inline]
        pub fn r#fw_version(&self) -> &::heapless::String<64> {
            &self.r#fw_version
        }
        /// Return a mutable reference to `fw_version`
        #[inline]
        pub fn mut_fw_version(&mut self) -> &mut ::heapless::String<64> {
            &mut self.r#fw_version
        }
        /// Set the value of `fw_version`
        #[inline]
        pub fn set_fw_version(&mut self, value: ::heapless::String<64>) -> &mut Self {
            self.r#fw_version = value.into();
            self
        }
        /// Builder method that sets the value of `fw_version`. Useful for initializing the message.
        #[inline]
        pub fn init_fw_version(mut self, value: ::heapless::String<64>) -> Self {
            self.r#fw_version = value.into();
            self
        }
        /// Return a reference to `serial`
        #[inline]
        pub fn r#serial(&self) -> &u32 {
            &self.r#serial
        }
        /// Return a mutable reference to `serial`
        #[inline]
        pub fn mut_serial(&mut self) -> &mut u32 {
            &mut self.r#serial
        }
        /// Set the value of `serial`
        #[inline]
        pub fn set_serial(&mut self, value: u32) -> &mut Self {
            self.r#serial = value.into();
            self
        }
        /// Builder method that sets the value of `serial`. Useful for initializing the message.
        #[inline]
        pub fn init_serial(mut self, value: u32) -> Self {
            self.r#serial = value.into();
            self
        }
    }
    impl ::micropb::MessageDecode for DeviceInfo {
        fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
            &mut self,
            decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
            len: usize,
        ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
            use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
            let before = decoder.bytes_read();
            while decoder.bytes_read() - before < len {
                let tag = decoder.decode_tag()?;
                match tag.field_num() {
                    0 => return Err(::micropb::DecodeError::ZeroField),
                    1u32 => {
                        let mut_ref = &mut self.r#fw_version;
                        {
                            decoder
                                .decode_string(mut_ref, ::micropb::Presence::Implicit)?;
                        };
                    }
                    2u32 => {
                        let mut_ref = &mut self.r#serial;
                        {
                            let val = decoder.decode_fixed32()?;
                            let val_ref = &val;
                            if *val_ref != 0 {
                                *mut_ref = val as _;
                            }
                        };
                    }
                    _ => {
                        decoder.skip_wire_value(tag.wire_type())?;
                    }
                }
            }
            Ok(())
        }
    }
    impl ::micropb::MessageEncode for DeviceInfo {
        const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::core::result::Result::Ok(65usize), | size | size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::core::result::Result::Ok(4usize), | size | size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        };
        fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
            &self,
            encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
        ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
            use ::micropb::{PbMap, FieldEncode};
            {
                let val_ref = &self.r#fw_version;
                if !val_ref.is_empty() {
                    encoder.encode_varint32(10u32)?;
                    encoder.encode_string(val_ref)?;
                }
            }
            {
                let val_ref = &self.r#serial;
                if *val_ref != 0 {
                    encoder.encode_varint32(21u32)?;
                    encoder.encode_fixed32(*val_ref as _)?;
                }
            }
            Ok(())
        }
        fn compute_size(&self) -> usize {
            use ::micropb::{PbMap, FieldEncode};
            let mut size = 0;
            {
                let val_ref = &self.r#fw_version;
                if !val_ref.is_empty() {
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
                }
            }
            {
                let val_ref = &self.r#serial;
                if *val_ref != 0 {
                    size += 1usize + 4;
                }
            }
            size
        }
    }
    #[derive(Debug, Default, PartialEq, Clone, Copy)]
    pub struct Command {
        pub r#id: u32,
        pub r#body: ::core::option::Option<Command_::Body>,
    }
    impl Command {
        /// Return a reference to `id`
        #[inline]
        pub fn r#id(&self) -> &u32 {
            &self.r#id
        }
        /// Return a mutable reference to `id`
        #[inline]
        pub fn mut_id(&mut self) -> &mut u32 {
            &mut self.r#id
        }
        /// Set the value of `id`
        #[inline]
        pub fn set_id(&mut self, value: u32) -> &mut Self {
            self.r#id = value.into();
            self
        }
        /// Builder method that sets the value of `id`. Useful for initializing the message.
        #[inline]
        pub fn init_id(mut self, value: u32) -> Self {
            self.r#id = value.into();
            self
        }
    }
    impl ::micropb::MessageDecode for Command {
        fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
            &mut self,
            decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
            len: usize,
        ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
            use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
            let before = decoder.bytes_read();
            while decoder.bytes_read() - before < len {
                let tag = decoder.decode_tag()?;
                match tag.field_num() {
                    0 => return Err(::micropb::DecodeError::ZeroField),
                    1u32 => {
                        let mut_ref = &mut self.r#id;
                        {
                            let val = decoder.decode_varint32()?;
                            let val_ref = &val;
                            if *val_ref != 0 {
                                *mut_ref = val as _;
                            }
                        };
                    }
                    2u32 => {
                        let mut_ref = loop {
                            if let ::core::option::Option::Some(variant) = &mut self
                                .r#body
                            {
                                if let Command_::Body::SetLed(variant) = &mut *variant {
                                    break &mut *variant;
                                }
                            }
                            self.r#body = ::core::option::Option::Some(
                                Command_::Body::SetLed(::core::default::Default::default()),
                            );
                        };
                        mut_ref.decode_len_delimited(decoder)?;
                    }
                    3u32 => {
                        let mut_ref = loop {
                            if let ::core::option::Option::Some(variant) = &mut self
                                .r#body
                            {
                                if let Command_::Body::Reboot(variant) = &mut *variant {
                                    break &mut *variant;
                                }
                            }
                            self.r#body = ::core::option::Option::Some(
                                Command_::Body::Reboot(::core::default::Default::default()),
                            );
                        };
                        mut_ref.decode_len_delimited(decoder)?;
                    }
                    _ => {
                        decoder.skip_wire_value(tag.wire_type())?;
                    }
                }
            }
            Ok(())
        }
    }
    impl ::micropb::MessageEncode for Command {
        const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::core::result::Result::Ok(5usize), | size | size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match 'oneof: {
                let mut max_size = 0;
                match ::micropb::const_map!(
                    ::micropb::const_map!(< SetLed as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        if size > max_size {
                            max_size = size;
                        }
                    }
                    ::core::result::Result::Err(err) => {
                        break 'oneof (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                match ::micropb::const_map!(
                    ::micropb::const_map!(< Reboot as ::micropb::MessageEncode >
                    ::MAX_SIZE, | size | ::micropb::size::sizeof_len_record(size)), |
                    size | size + 1usize
                ) {
                    ::core::result::Result::Ok(size) => {
                        if size > max_size {
                            max_size = size;
                        }
                    }
                    ::core::result::Result::Err(err) => {
                        break 'oneof (::core::result::Result::<usize, _>::Err(err));
                    }
                }
                ::core::result::Result::Ok(max_size)
            } {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        };
        fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
            &self,
            encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
        ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
            use ::micropb::{PbMap, FieldEncode};
            {
                let val_ref = &self.r#id;
                if *val_ref != 0 {
                    encoder.encode_varint32(8u32)?;
                    encoder.encode_varint32(*val_ref as _)?;
                }
            }
            if let Some(oneof) = &self.r#body {
                match &*oneof {
                    Command_::Body::SetLed(val_ref) => {
                        let val_ref = &*val_ref;
                        encoder.encode_varint32(18u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                    Command_::Body::Reboot(val_ref) => {
                        let val_ref = &*val_ref;
                        encoder.encode_varint32(26u32)?;
                        val_ref.encode_len_delimited(encoder)?;
                    }
                }
            }
            Ok(())
        }
        fn compute_size(&self) -> usize {
            use ::micropb::{PbMap, FieldEncode};
            let mut size = 0;
            {
                let val_ref = &self.r#id;
                if *val_ref != 0 {
                    size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
                }
            }
            if let Some(oneof) = &self.r#body {
                match &*oneof {
                    Command_::Body::SetLed(val_ref) => {
                        let val_ref = &*val_ref;
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                    Command_::Body::Reboot(val_ref) => {
                        let val_ref = &*val_ref;
                        size
                            += 1usize
                                + ::micropb::size::sizeof_len_record(
                                    val_ref.compute_size(),
                                );
                    }
                }
            }
            size
        }
    }
    /// Inner types for `Command`
    pub mod Command_ {
        #[derive(Debug, PartialEq, Clone, Copy)]
        pub enum Body {
            SetLed(super::SetLed),
            Reboot(super::Reboot),
        }
    }
    #[derive(Debug, Default, PartialEq, Clone, Copy)]
    pub struct SetLed {
        pub r#on: bool,
        pub r#rgb: u32,
    }
    impl SetLed {
        /// Return a reference to `on`
        #[inline]
        pub fn r#on(&self) -> &bool {
            &self.r#on
        }
        /// Return a mutable reference to `on`
        #[inline]
        pub fn mut_on(&mut self) -> &mut bool {
            &mut self.r#on
        }
        /// Set the value of `on`
        #[inline]
        pub fn set_on(&mut self, value: bool) -> &mut Self {
            self.r#on = value.into();
            self
        }
        /// Builder method that sets the value of `on`. Useful for initializing the message.
        #[inline]
        pub fn init_on(mut self, value: bool) -> Self {
            self.r#on = value.into();
            self
        }
        /// Return a reference to `rgb`
        #[inline]
        pub fn r#rgb(&self) -> &u32 {
            &self.r#rgb
        }
        /// Return a mutable reference to `rgb`
        #[inline]
        pub fn mut_rgb(&mut self) -> &mut u32 {
            &mut self.r#rgb
        }
        /// Set the value of `rgb`
        #[inline]
        pub fn set_rgb(&mut self, value: u32) -> &mut Self {
            self.r#rgb = value.into();
            self
        }
        /// Builder method that sets the value of `rgb`. Useful for initializing the message.
        #[inline]
        pub fn init_rgb(mut self, value: u32) -> Self {
            self.r#rgb = value.into();
            self
        }
    }
    impl ::micropb::MessageDecode for SetLed {
        fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
            &mut self,
            decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
            len: usize,
        ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
            use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
            let before = decoder.bytes_read();
            while decoder.bytes_read() - before < len {
                let tag = decoder.decode_tag()?;
                match tag.field_num() {
                    0 => return Err(::micropb::DecodeError::ZeroField),
                    1u32 => {
                        let mut_ref = &mut self.r#on;
                        {
                            let val = decoder.decode_bool()?;
                            let val_ref = &val;
                            if *val_ref {
                                *mut_ref = val as _;
                            }
                        };
                    }
                    2u32 => {
                        let mut_ref = &mut self.r#rgb;
                        {
                            let val = decoder.decode_varint32()?;
                            let val_ref = &val;
                            if *val_ref != 0 {
                                *mut_ref = val as _;
                            }
                        };
                    }
                    _ => {
                        decoder.skip_wire_value(tag.wire_type())?;
                    }
                }
            }
            Ok(())
        }
    }
    impl ::micropb::MessageEncode for SetLed {
        const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::core::result::Result::Ok(1usize), | size | size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::core::result::Result::Ok(5usize), | size | size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        };
        fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
            &self,
            encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
        ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
            use ::micropb::{PbMap, FieldEncode};
            {
                let val_ref = &self.r#on;
                if *val_ref {
                    encoder.encode_varint32(8u32)?;
                    encoder.encode_bool(*val_ref)?;
                }
            }
            {
                let val_ref = &self.r#rgb;
                if *val_ref != 0 {
                    encoder.encode_varint32(16u32)?;
                    encoder.encode_varint32(*val_ref as _)?;
                }
            }
            Ok(())
        }
        fn compute_size(&self) -> usize {
            use ::micropb::{PbMap, FieldEncode};
            let mut size = 0;
            {
                let val_ref = &self.r#on;
                if *val_ref {
                    size += 1usize + 1;
                }
            }
            {
                let val_ref = &self.r#rgb;
                if *val_ref != 0 {
                    size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
                }
            }
            size
        }
    }
    #[derive(Debug, Default, PartialEq, Clone, Copy)]
    pub struct Reboot {}
    impl Reboot {}
    impl ::micropb::MessageDecode for Reboot {
        fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
            &mut self,
            decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
            len: usize,
        ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
            use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
            let before = decoder.bytes_read();
            while decoder.bytes_read() - before < len {
                let tag = decoder.decode_tag()?;
                match tag.field_num() {
                    0 => return Err(::micropb::DecodeError::ZeroField),
                    _ => {
                        decoder.skip_wire_value(tag.wire_type())?;
                    }
                }
            }
            Ok(())
        }
    }
    impl ::micropb::MessageEncode for Reboot {
        const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
            let mut max_size = 0;
            ::core::result::Result::Ok(max_size)
        };
        fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
            &self,
            encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
        ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
            use ::micropb::{PbMap, FieldEncode};
            Ok(())
        }
        fn compute_size(&self) -> usize {
            use ::micropb::{PbMap, FieldEncode};
            let mut size = 0;
            size
        }
    }
    #[derive(Debug, Default, PartialEq, Clone)]
    pub struct Event {
        pub r#id: u32,
        pub r#data: ::heapless::Vec<u8, 64>,
    }
    impl Event {
        /// Return a reference to `id`
        #[inline]
        pub fn r#id(&self) -> &u32 {
            &self.r#id
        }
        /// Return a mutable reference to `id`
        #[inline]
        pub fn mut_id(&mut self) -> &mut u32 {
            &mut self.r#id
        }
        /// Set the value of `id`
        #[inline]
        pub fn set_id(&mut self, value: u32) -> &mut Self {
            self.r#id = value.into();
            self
        }
        /// Builder method that sets the value of `id`. Useful for initializing the message.
        #[inline]
        pub fn init_id(mut self, value: u32) -> Self {
            self.r#id = value.into();
            self
        }
        /// Return a reference to `data`
        #[inline]
        pub fn r#data(&self) -> &::heapless::Vec<u8, 64> {
            &self.r#data
        }
        /// Return a mutable reference to `data`
        #[inline]
        pub fn mut_data(&mut self) -> &mut ::heapless::Vec<u8, 64> {
            &mut self.r#data
        }
        /// Set the value of `data`
        #[inline]
        pub fn set_data(&mut self, value: ::heapless::Vec<u8, 64>) -> &mut Self {
            self.r#data = value.into();
            self
        }
        /// Builder method that sets the value of `data`. Useful for initializing the message.
        #[inline]
        pub fn init_data(mut self, value: ::heapless::Vec<u8, 64>) -> Self {
            self.r#data = value.into();
            self
        }
    }
    impl ::micropb::MessageDecode for Event {
        fn decode<IMPL_MICROPB_READ: ::micropb::PbRead>(
            &mut self,
            decoder: &mut ::micropb::PbDecoder<IMPL_MICROPB_READ>,
            len: usize,
        ) -> Result<(), ::micropb::DecodeError<IMPL_MICROPB_READ::Error>> {
            use ::micropb::{PbBytes, PbString, PbVec, PbMap, FieldDecode};
            let before = decoder.bytes_read();
            while decoder.bytes_read() - before < len {
                let tag = decoder.decode_tag()?;
                match tag.field_num() {
                    0 => return Err(::micropb::DecodeError::ZeroField),
                    1u32 => {
                        let mut_ref = &mut self.r#id;
                        {
                            let val = decoder.decode_varint32()?;
                            let val_ref = &val;
                            if *val_ref != 0 {
                                *mut_ref = val as _;
                            }
                        };
                    }
                    2u32 => {
                        let mut_ref = &mut self.r#data;
                        {
                            decoder
                                .decode_bytes(mut_ref, ::micropb::Presence::Implicit)?;
                        };
                    }
                    _ => {
                        decoder.skip_wire_value(tag.wire_type())?;
                    }
                }
            }
            Ok(())
        }
    }
    impl ::micropb::MessageEncode for Event {
        const MAX_SIZE: ::core::result::Result<usize, &'static str> = 'msg: {
            let mut max_size = 0;
            match ::micropb::const_map!(
                ::core::result::Result::Ok(5usize), | size | size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            match ::micropb::const_map!(
                ::core::result::Result::Ok(65usize), | size | size + 1usize
            ) {
                ::core::result::Result::Ok(size) => {
                    max_size += size;
                }
                ::core::result::Result::Err(err) => {
                    break 'msg (::core::result::Result::<usize, _>::Err(err));
                }
            }
            ::core::result::Result::Ok(max_size)
        };
        fn encode<IMPL_MICROPB_WRITE: ::micropb::PbWrite>(
            &self,
            encoder: &mut ::micropb::PbEncoder<IMPL_MICROPB_WRITE>,
        ) -> Result<(), IMPL_MICROPB_WRITE::Error> {
            use ::micropb::{PbMap, FieldEncode};
            {
                let val_ref = &self.r#id;
                if *val_ref != 0 {
                    encoder.encode_varint32(8u32)?;
                    encoder.encode_varint32(*val_ref as _)?;
                }
            }
            {
                let val_ref = &self.r#data;
                if !val_ref.is_empty() {
                    encoder.encode_varint32(18u32)?;
                    encoder.encode_bytes(val_ref)?;
                }
            }
            Ok(())
        }
        fn compute_size(&self) -> usize {
            use ::micropb::{PbMap, FieldEncode};
            let mut size = 0;
            {
                let val_ref = &self.r#id;
                if *val_ref != 0 {
                    size += 1usize + ::micropb::size::sizeof_varint32(*val_ref as _);
                }
            }
            {
                let val_ref = &self.r#data;
                if !val_ref.is_empty() {
                    size += 1usize + ::micropb::size::sizeof_len_record(val_ref.len());
                }
            }
            size
        }
    }
}
