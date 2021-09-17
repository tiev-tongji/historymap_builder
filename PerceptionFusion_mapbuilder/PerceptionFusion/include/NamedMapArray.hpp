/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __NamedMapArray_hpp__
#define __NamedMapArray_hpp__

#include <string>
#include <vector>


class NamedMapArray
{
    public:
        std::string name;

        int16_t    height;

        int16_t    width;

        std::vector< std::vector< uint8_t > > cells;

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~NamedMapArray() {}

        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void* buf, uint32_t offset, uint32_t maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline uint32_t getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void* buf, uint32_t offset, uint32_t maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "NamedMapArray"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int NamedMapArray::encode(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = (int64_t)getHash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int NamedMapArray::decode(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t NamedMapArray::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t NamedMapArray::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* NamedMapArray::getTypeName()
{
    return "NamedMapArray";
}

int NamedMapArray::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;

    char* name_cstr = (char*) this->name.c_str();
    thislen = __string_encode_array(buf, offset + pos, maxlen - pos, &name_cstr, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->height, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->width, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    if(this->width > 0) {
        for (int a0 = 0; a0 < this->height; ++a0) {
            thislen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->cells[a0][0], this->width);
            if(thislen < 0) return thislen; else pos += thislen;
        }
    }

    return pos;
}

int NamedMapArray::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int32_t __name_len__;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__name_len__, 1);
    if(thislen < 0) return thislen; else pos += thislen;
    if((uint32_t)__name_len__ > maxlen - pos) return -1;
    this->name.assign(((const char*)buf) + offset + pos, __name_len__ - 1);
    pos += __name_len__;

    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->height, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->width, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    this->cells.resize(this->height);
    for (int a0 = 0; a0 < this->height; ++a0) {
        if(this->width > 0) {
            this->cells[a0].resize(this->width);
            thislen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->cells[a0][0], this->width);
            if(thislen < 0) return thislen; else pos += thislen;
        }
    }

    return pos;
}

uint32_t NamedMapArray::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += this->name.size() + 4 + 1;
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += this->height * __byte_encoded_array_size(NULL, this->width);
    return enc_size;
}

uint64_t NamedMapArray::_computeHash(const __zcm_hash_ptr*)
{
    uint64_t hash = (uint64_t)0xa31a5b607296cf44LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
