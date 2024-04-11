#include "EchonetLite.hpp"

class HousingFacilitiesDeviceClass : public EchonetLite {
  public:
    enum class ClassCode : uint8_t {
        LowVoltageSmartElectricMeter = 0x88, // 低圧スマート電力量メータ
    };

    template <class PropertyType>
    void generateGetRequest(const std::vector<PropertyType> &property) {
        EchonetLite::generateGetRequest(property);
        this->data.EDATA.DEOJ.classGroupCode = EchonetLite::ClassGroupCode::HousingFacilitiesDeviceClassGroup;
    }
};