/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __MsgRoadMarkingList_hpp__
#define __MsgRoadMarkingList_hpp__

#include <vector>
#include "Lane.hpp"
#include "StopLine.hpp"
#include "Zebra.hpp"
#include "Curb.hpp"
#include "NoParking.hpp"
#include "Chevron.hpp"


class MsgRoadMarkingList
{
    public:
        int32_t    current_lane_id;

        int32_t    num;

        std::vector< Lane > lanes;

        StopLine   stop_line;

        Zebra      zebra;

        Curb       curb;

        NoParking  no_parking;

        Chevron    chevron;

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~MsgRoadMarkingList() {}

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
         * Returns "MsgRoadMarkingList"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int MsgRoadMarkingList::encode(void* buf, uint32_t offset, uint32_t maxlen) const
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

int MsgRoadMarkingList::decode(const void* buf, uint32_t offset, uint32_t maxlen)
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

uint32_t MsgRoadMarkingList::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t MsgRoadMarkingList::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* MsgRoadMarkingList::getTypeName()
{
    return "MsgRoadMarkingList";
}

int MsgRoadMarkingList::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->current_lane_id, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    for (int a0 = 0; a0 < this->num; ++a0) {
        thislen = this->lanes[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(thislen < 0) return thislen; else pos += thislen;
    }

    thislen = this->stop_line._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->zebra._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->curb._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->no_parking._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->chevron._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int MsgRoadMarkingList::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->current_lane_id, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    this->lanes.resize(this->num);
    for (int a0 = 0; a0 < this->num; ++a0) {
        thislen = this->lanes[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(thislen < 0) return thislen; else pos += thislen;
    }

    thislen = this->stop_line._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->zebra._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->curb._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->no_parking._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->chevron._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t MsgRoadMarkingList::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num; ++a0) {
        enc_size += this->lanes[a0]._getEncodedSizeNoHash();
    }
    enc_size += this->stop_line._getEncodedSizeNoHash();
    enc_size += this->zebra._getEncodedSizeNoHash();
    enc_size += this->curb._getEncodedSizeNoHash();
    enc_size += this->no_parking._getEncodedSizeNoHash();
    enc_size += this->chevron._getEncodedSizeNoHash();
    return enc_size;
}

uint64_t MsgRoadMarkingList::_computeHash(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == MsgRoadMarkingList::getHash)
            return 0;
    const __zcm_hash_ptr cp = { p, (void*)MsgRoadMarkingList::getHash };

    uint64_t hash = (uint64_t)0x8f039b04b6ef324aLL +
         Lane::_computeHash(&cp) +
         StopLine::_computeHash(&cp) +
         Zebra::_computeHash(&cp) +
         Curb::_computeHash(&cp) +
         NoParking::_computeHash(&cp) +
         Chevron::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
