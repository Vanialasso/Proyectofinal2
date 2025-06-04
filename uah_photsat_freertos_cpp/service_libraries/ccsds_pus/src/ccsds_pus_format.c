#include "public/ccsds_pus.h"
#include "public/serialize.h"

void ccsds_pus_tc_read_fields(uint8_t tc_bytes[],
		ccsds_packet_primary_header_t * p_tc_packet_header,
        ccsds_pus_tc_df_header_t * p_tc_df_header,
        uint16_t * p_tc_packet_err_ctrl) {

    // Deserialize Packet ID and store it into field packet_id
    p_tc_packet_header->packet_id = deserialize_uint16(&tc_bytes[0]);

    // Deserialize Packet Seq. Control and store it into field packet_seq_ctrl
    p_tc_packet_header->packet_seq_ctrl = deserialize_uint16(&tc_bytes[2]);

    // Deserialize Packet Seq. Control and store it into field packet_length
    p_tc_packet_header->packet_length = deserialize_uint16(&tc_bytes[4]);

    // Deserialize Data Field Header

    // Read the MSB of the Data Field Header and store it into flag_ver_ack
    p_tc_df_header->version_ackflags = tc_bytes[6];

    // Read the remaining fields of the Data Field Header into the struct
    p_tc_df_header->type = tc_bytes[7];
    p_tc_df_header->subtype = tc_bytes[8];
    p_tc_df_header->sourceID = deserialize_uint16(&tc_bytes[9]);

    // Deserialize Packet Error Control and store it at p_tc_packet_err_ctrl
    *p_tc_packet_err_ctrl = deserialize_uint16(
    	&tc_bytes[CCSDS_PRIMARY_HEADER_SIZE + p_tc_packet_header->packet_length -1]);

}

void ccsds_pus_tm_write_fields(uint8_t tm_bytes[],
        const struct ccsds_packet_primary_header * p_tm_packet_header,
        const struct ccsds_pus_tm_df_header * p_tm_df_header) {

    serialize_uint16(p_tm_packet_header->packet_id, &tm_bytes[0]);

    // Serialize Packet Sequence Control from packet_seq_ctrl
    serialize_uint16(p_tm_packet_header->packet_seq_ctrl, &tm_bytes[2]);

    // Serialize Packet Length from packet_length
    serialize_uint16(p_tm_packet_header->packet_length, &tm_bytes[4]);

    // Store version field into the corresponding byte
    tm_bytes[6] = p_tm_df_header->version_obtstatus;

    // Store the remaining fields into their respective locations
    tm_bytes[7] = p_tm_df_header->type;
    tm_bytes[8] = p_tm_df_header->subtype;
    serialize_uint16(p_tm_df_header->msgtype_counter, &tm_bytes[9]);

    serialize_uint16(p_tm_df_header->destinationID, &tm_bytes[11]);

    serialize_uint32(p_tm_df_header->obt_secs,&tm_bytes[13]);

    serialize_uint16(p_tm_df_header->obt_finetime,&tm_bytes[17]);


}
