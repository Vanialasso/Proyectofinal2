#ifndef INCLUDE__CCSDS_PUS_FORMAT_H_
#define INCLUDE__CCSDS_PUS_FORMAT_H_

#include "public/basic_types.h"



//!CCSDS Primary Header Size
#define CCSDS_PRIMARY_HEADER_SIZE 6

/**
 * \brief Structure type to store the CCSDS packet primary header.
 */
struct __attribute ((__packed__)) ccsds_packet_primary_header {

    uint16_t packet_id;
    uint16_t packet_seq_ctrl;
    uint16_t packet_length;

};
typedef struct ccsds_packet_primary_header ccsds_packet_primary_header_t;

/**
 * \brief Structure type to store the PUS TC Data Field Header
 */
struct __attribute ((__packed__))  ccsds_pus_tc_df_header {

    uint8_t version_ackflags;
    uint8_t type;
    uint8_t subtype;
    uint16_t sourceID;

};
typedef struct ccsds_pus_tc_df_header ccsds_pus_tc_df_header_t;


/**
 * \brief Structure type to store the Data Field Header field of a TM packet.
 */
struct __attribute ((__packed__))  ccsds_pus_tm_df_header {

    uint8_t version_obtstatus;
    uint8_t type;
    uint8_t subtype;
    uint16_t msgtype_counter;
    uint16_t destinationID;
    uint32_t obt_secs;
    uint16_t obt_finetime;

};
typedef struct ccsds_pus_tm_df_header ccsds_pus_tm_df_header_t;


/**
 * \brief Get APID from a telecommand's Packet ID.
 *
 * \param tc_packet_id the telecommand's Packet ID
 *
 * \return the telecommand's APID
 */
#define ccsds_pus_tc_get_APID(tc_packet_id) ((tc_packet_id) & 0x07FF)

/**
 * \brief Get Sequence Count from a telecommand's Packet Sequence Control.
 *
 * \param tc_packet_seq_ctrl the telecommand's Packet Sequence Control
 *
 * \return the telecommand's Sequence Flags
 */
#define ccsds_pus_tc_get_seq_flags(tc_packet_seq_ctrl) (((tc_packet_seq_ctrl) & 0xC000) >> 14)

/**
 * \brief Get Sequence Count from a telecommand's Packet Sequence Control.
 *
 * \param tc_packet_seq_ctrl the telecommand's Packet Sequence Control
 *
 * \return the telecommand's Sequence Count
 */
#define ccsds_pus_tmtc_get_seq_count(tmtc_packet_seq_ctrl) ((tmtc_packet_seq_ctrl) & 0x3FFF)

/**
 * \brief Get Ack from a telecommand's Data Field Header.
 *
 * \param tc_df_header the telecommand's Data Field Header
 *
 * \return the telecommand's Ack
 */
#define ccsds_pus_tc_get_ack(flag_ver_ack) ((flag_ver_ack) & 0x0F)

/**
 * \brief Deserializes the fields of a telecommand stored in a vector
 *
 * \param tc_bytes vector that stores the bytes of the telecommand
 * \param p_tc_packet_header pointer to the struct that shall store the Packet
 *          Header
 * \param p_tc_df_header pointer to the struct that shall store the Data Field
 *                    Header
 * \param p_tc_packet_err_ctrl pointer to the variable that shall store the
 *                          Packet Error Control
 */

void ccsds_pus_tc_read_fields(uint8_t tc_bytes[],
					   ccsds_packet_primary_header_t *p_tc_packet_header,
                       ccsds_pus_tc_df_header_t *p_tc_df_header,
                        uint16_t * p_tc_packet_err_ctrl);


/**
 * \brief Builds the Packet ID of a telemetry.
 *
 * \param apid the value of the APID subfield
 *
 * \return the Packet ID field
 */
#define ccsds_pus_tm_build_packet_id(apid) ((1 << 11) | ((apid) & 0x07FF))

/**
 * \brief Builds the Packet Sequence Control of a telmetry.
 *
 * \param flags the value of the Sequence Flags subfield
 * \param count the value of the Sequence Count subfield
 *
 * \return the Packet Sequence Control field
 */
#define ccsds_pus_tm_build_packet_seq_ctrl(flags, count) \
    ((((flags) & 0x3) << 14) | ((count) & 0x3FFF))

/**
 * \brief Builds the most significant byte of the Data Field Header of a telmetry.
 *
 * \param version the value of the Version subfield
 *
 * \return the MSB of the Data Field Header
 */
#define ccsds_pus_tm_build_df_header_version(version) (((version) & 0xF) << 4)

/**
 * \brief Serializes and stores into a vector the main fields of a TM packet.
 *
 * \param tm_bytes vector that will store the generated serialized telemetry
 * \param p_tm_packet_header const pointer to the struct that defines the
 *                           Packet Header
 * \param p_tm_df_header the const pointer to the struct that defines the
 *                           TM Data Field Header
 */
void ccsds_pus_tm_write_fields(uint8_t tm_bytes[],
                        const ccsds_packet_primary_header_t * p_tm_packet_header,
                        const ccsds_pus_tm_df_header_t * p_tm_df_header);

#endif /* INCLUDE_CCSDS_PUS_FORMAT_H_ */
