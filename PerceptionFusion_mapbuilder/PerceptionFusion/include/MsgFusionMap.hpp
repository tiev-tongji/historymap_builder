/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __MsgFusionMap_hpp__
#define __MsgFusionMap_hpp__

#include <vector>


class MsgFusionMap
{
    public:
        int64_t    time_stamp;

        double     car_utm_position_x;

        double     car_utm_position_y;

        float      car_heading;

        float      map_resolution;

        int16_t    map_row_num;

        int16_t    map_column_num;

        int16_t    car_center_column;

        int16_t    car_center_row;

        std::vector< std::vector< uint8_t > > map_cells;

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~MsgFusionMap() {}

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
         * Returns "MsgFusionMap"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int MsgFusionMap::encode(void* buf, uint32_t offset, uint32_t maxlen) const
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

int MsgFusionMap::decode(const void* buf, uint32_t offset, uint32_t maxlen)
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

uint32_t MsgFusionMap::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t MsgFusionMap::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* MsgFusionMap::getTypeName()
{
    return "MsgFusionMap";
}

int MsgFusionMap::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->time_stamp, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->car_utm_position_x, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->car_utm_position_y, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->car_heading, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->map_resolution, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->map_row_num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->map_column_num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->car_center_column, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->car_center_row, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    if(this->map_column_num > 0) {
        for (int a0 = 0; a0 < this->map_row_num; ++a0) {
            thislen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->map_cells[a0][0], this->map_column_num);
            if(thislen < 0) return thislen; else pos += thislen;
        }
    }

    return pos;
}

int MsgFusionMap::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->time_stamp, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->car_utm_position_x, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->car_utm_position_y, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->car_heading, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->map_resolution, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->map_row_num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->map_column_num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->car_center_column, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->car_center_row, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    this->map_cells.resize(this->map_row_num);
    for (int a0 = 0; a0 < this->map_row_num; ++a0) {
        if(this->map_column_num > 0) {
            this->map_cells[a0].resize(this->map_column_num);
            thislen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->map_cells[a0][0], this->map_column_num);
            if(thislen < 0) return thislen; else pos += thislen;
        }
    }

    return pos;
}

uint32_t MsgFusionMap::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += this->map_row_num * __byte_encoded_array_size(NULL, this->map_column_num);
    return enc_size;
}

uint64_t MsgFusionMap::_computeHash(const __zcm_hash_ptr*)
{
    uint64_t hash = (uint64_t)0xf4f8bd029e1661d6LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif