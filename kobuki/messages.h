namespace kobuki {

struct PacketHeader {
    const uint16_t HEADER = 0xAA55;
    uint8_t length;
} __attribute__((__packed__));

enum class Commands : uint8_t {
    Motion = 1,
    Reserved_1,
    Sound,
    SoundSequence,
    Reserved_2,
    Reserved_3,
    Reserved_4,
    Reserved_5,
    RequestExtra,
    Reserved_6,
    Reserved_7,
    GeneralPurposeOutput,
    SetControllerGain,
    GetControllerGain,
};

static_assert(sizeof(PacketHeader) == 3, "Unexpected size");

struct CommandSubPayloadHeader {
    Commands type;
    uint8_t length;
} __attribute__((__packed__));

static_assert(sizeof(CommandSubPayloadHeader) == 2, "Unexpected size");

struct MotionMessage : CommandSubPayloadHeader {
    int16_t velocity;
    int16_t radius;
} __attribute__((__packed__));

static_assert(sizeof(MotionMessage) == sizeof(CommandSubPayloadHeader) + 4, "Unexpected size");

struct SoundMessage : CommandSubPayloadHeader {
    uint16_t frequency;
    uint8_t duration;
} __attribute__((__packed__));

static_assert(sizeof(SoundMessage) == sizeof(CommandSubPayloadHeader) + 3, "Unexpected size");

enum class SoundSequenceNumber : uint8_t {
    ON = 0,
    OFF,
    RECHARGE,
    BUTTON,
    ERROR,
    CLEANING_START,
    CLEANING_STOP,
};

struct SoundSequence : CommandSubPayloadHeader {
    SoundSequenceNumber sequence_number;
} __attribute__((__packed__));

static_assert(sizeof(SoundSequence) == sizeof(CommandSubPayloadHeader) + 1, "Unexpected size");

enum class DigitalOutput : uint16_t {
    CHANNEL_0    = 0x0001,
    CHANNEL_1    = 0x0002,
    CHANNEL_3    = 0x0004,
    CHANNEL_4    = 0x0008,
    POWER_3_3    = 0x0010,
    POWER_5      = 0x0020,
    POWER_12_5   = 0x0040,
    POWER_12_1_5 = 0x0080,
    LED_1        = 0x0100,
    LED_2        = 0x0200,
    LED_3        = 0x0400,
    LED_4        = 0x0800,
};

struct RequestExtra : CommandSubPayloadHeader {
    DigitalOutput digital_output;
} __attribute__((__packed__));

static_assert(sizeof(RequestExtra) == sizeof(CommandSubPayloadHeader) + 2, "Unexpected size");

enum class GainType : uint8_t {
    DEFAULT = 0,
    USER = 1,
};

struct SetControllerGain : CommandSubPayloadHeader {
    GainType type;
    int32_t proportional;
    int32_t integral;
    int32_t derivate;
} __attribute__((__packed__));

static_assert(sizeof(SetControllerGain) == sizeof(CommandSubPayloadHeader) + 13, "Unexpected size");

struct GetControllerGain : CommandSubPayloadHeader {
    char unused;
} __attribute__((__packed__));

static_assert(sizeof(GetControllerGain) == sizeof(CommandSubPayloadHeader) + 1, "Unexpected size");

enum class Feedback : uint8_t {
    BasicSensorData = 1,
    Reserved_1,
    DockingIR,
    InertialSensor,
    Cliff,
    Current,
    Reserved_2,
    Reserved_3,
    Reserved_4,
    HardwareVersion,
    FirmwareVersion,
    Reserved_5,
    RawData3AxisGyro,
    Reserved_6,
    Reserved_7,
    GeneralPurposeInput,
    Reserved_8,
    Reserved_9,
    UDID,
    Reserved_10,
    ControllerInfo,
};

struct FeedbackSubPayloadHeader {
    Feedback type;
    uint8_t length;
} __attribute__((__packed__));

static_assert(sizeof(FeedbackSubPayloadHeader) == 2, "Unexpected size");

enum class Side : uint8_t {
    Right  = 0x01,
    Center = 0x02,
    Left   = 0x04,
};

enum class Wheel : uint8_t {
    Right = 0x01,
    Left  = 0x02,
};

enum class Button : uint8_t {
    Button_0 = 0x01,
    Button_1 = 0x02,
    Button_2 = 0x04,
};

enum class Charger : uint8_t {
    Discharging = 0,
    DockingCharged = 2,
    DockingCharging = 6,
    AdpaterCharged = 18,
    AdapterCharging = 22,
};

struct BasicSensorData : FeedbackSubPayloadHeader {
    uint16_t timestamp_ms,
    Side bumper;
    Wheel wheel_drop;
    Side cliff;
    uint16_t left_encoder;
    uint16_t right_encoder;
    int8_t left_pwm;
    int8_t right_pwm;
    Button button;
    Charger charger;
    uint8_t battery_voltage;
    Wheel overcurrent;
} __attribute__((__packed__));

static_assert(sizeof(BasicSensorData) == sizeof(FeedbackSubPayloadHeader) + 15, "Unexpected size");

enum class Signal : uint8_t {
    NearLeft   = 0x01,
    NearCenter = 0x02,
    NearRight  = 0x04,
    FarCenter  = 0x08,
    FarLeft    = 0x10,
    FarRight   = 0x20,
};

struct DockingIR : FeedbackSubPayloadHeader {
    Signal right;
    Signal center;
    Signal left;
} __attribute__((__packed__));

static_assert(sizeof(DockingIR) == sizeof(FeedbackSubPayloadHeader) + 3, "Unexpected size");

struct InertialSensorData : FeedbackSubPayloadHeader {
    int16_t angle;
    uint16_t angle_rate;
    char unused[3];
}__attribute__((__packed__));

static_assert(sizeof(IntertialSensorData) == sizeof(FeedbackSubPayloadHeader) + 7, "Unexpected size");

struct CliffSensorData : FeedbackSubPayloadHeader {
    uint16_t right_voltage;
    uint16_t center_voltage;
    uint16_t left_voltage;
}__attribute__((__packed__));

static_assert(sizeof(CliffSensorData) == sizeof(FeedbackSubPayloadHeader) + 6, "Unexpected size");

struct Current : FeedbackSubPayloadHeader {
    int16_t left_current;
    int16_t right_current;
}__attribute__((__packed__));

static_assert(sizeof(Current) == sizeof(FeedbackSubPayloadHeader) + 4, "Unexpected size");

struct HardwareVersion : FeedbackSubPayloadHeader {
    uint8_t patch;
    uint8_t minor_1;
    uint8_t major_1;
    char unused;
}__attribute__((__packed__));

static_assert(sizeof(HardwareVersion) == sizeof(FeedbackSubPayloadHeader) + 4, "Unexpected size");

struct FirmwareVersion : FeedbackSubPayloadHeader {
    uint8_t patch;
    uint8_t minor_1;
    uint8_t major_1;
    char unused;
}__attribute__((__packed__));

static_assert(sizeof(FirmwareVersion) == sizeof(FeedbackSubPayloadHeader) + 4, "Unexpected size");

struct RawData3AxisGyro : FeedbackSubPayloadHeader {
    uint8_t frame_id;
    uint8_t n;
}__attribute__((__packed__));

static_assert(sizeof(RawData3AxisGyro) == sizeof(FeedbackSubPayloadHeader) + 2, "Unexpected size");

enum class DigitalInput : uint16_t {
    Channel_0 = 0x01,
    Channel_1 = 0x02,
    Channel_2 = 0x04,
    Channel_3 = 0x08,
};

struct GeneralPurposeInput : FeedbackSubPayloadHeader {
    DigitalInput digital_input;
    uint16_t analog_input_0;
    uint16_t analog_input_1;
    uint16_t analog_input_2;
    uint16_t analog_input_3;
    char unused[6];
}__attribute__((__packed__));

static_assert(sizeof(GeneralPurposeInput) == sizeof(FeedbackSubPayloadHeader) + 16, "Unexpected size");

struct UniqueDeviceIdentifier : FeedbackSubPayloadHeader {
    char id[12];
}__attribute__((__packed__));

static_assert(sizeof(UniqueDeviceIdentifier) == sizeof(FeedbackSubPayloadHeader) + 12, "Unexpected size");

struct ControllerInfo : FeedbackSubPayloadHeader {
    GainType type;
    int32_t proportional;
    int32_t integral;
    int32_t derivate;
}__attribute__((__packed__));

} // namespace kobuki
