#ifndef  __SPI_PLATFORM_H__
#define  __SPI_PLATFORM_H__

typedef struct {
  EFI_PHYSICAL_ADDRESS  DeviceBaseAddress;
  EFI_PHYSICAL_ADDRESS  RegionBaseAddress;
  UINT32                PageSize;
} SPI_CONFIGURATION_DATA;

#endif /* __SPI_PLATFORM_H__ */