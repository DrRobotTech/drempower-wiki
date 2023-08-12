#ifndef drempower_FORMAT_DATA_H
#define drempower_FORMAT_DATA_H
// note: the bit order is in accordance with intel little endian


enum
{
    OP_PP_MODE = 1, // INPUT_MODE_TRAP_TRAJ (Profile Position Mode)
    OP_PV_MODE = 3, // INPUT_MODE_VELOCITY_RAMP (Profile Velocity Mode)
    OP_PT_MODE = 4, // INPUT_MODE_TORQUE_RAMP (Profile Torque  Mode)
    OP_HZ_MODE = 6, // SET_ZERO_POSITION (Homing Mode)
    OP_TP_MODE = 7, // INPUT_MODE_POS_FILTER (Interpolated Position Mode)

    OP_FP_MODE = -1, // INPUT_MODE_FF_CONTROL(feedforward Position Mode)
    OP_FV_MODE = -3, // INPUT_MODE_FF_CONTROL(feedforward  Velocity Mode)
    OP_FT_MODE = -4, // INPUT_MODE_FF_CONTROL(feedforward  Torque  Mode)
    OP_IP_MODE = -7, // INPUT_MODE_IMPEDANCE_CONTROL(Impedance Control)
};

static inline void int8_to_data(int8_t val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
}

static inline void uint16_to_data(uint16_t val, uint8_t *data)
{
	data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}

static inline uint16_t data_to_uint16(uint8_t *data)
{
		uint16_t tmp_uint16;
		tmp_uint16 = (((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
		return tmp_uint16;
}


static inline void uint_to_data(uint32_t val, uint8_t *data)
{
	data[3] = (uint8_t)(val >> 24);
    data[2] = (uint8_t)(val >> 16);
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}

static inline uint32_t data_to_uint(uint8_t *data)
{
		uint32_t tmp_uint;
		tmp_uint = (((uint32_t)data[3] << 24) + ((uint32_t)data[2] << 16) + ((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
		return tmp_uint;
}

static inline void int16_to_data(int16_t val, uint8_t *data)
{
	data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
}

static inline int16_t data_to_int16(uint8_t *data)
{
	int16_t tmp_int16;
    *(((uint8_t*)(&tmp_int16)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int16)) + 1) = data[1];
    return tmp_int16;
}

static inline void int_to_data(int val, uint8_t *data)
{
	data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
	int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}

static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}
#endif // drempower_FORMAT_DATA_H