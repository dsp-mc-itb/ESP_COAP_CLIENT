
# Change Log

This template is based on https://keepachangelog.com/en/1.0.0/

## Added Branch
### riset-itb
Code that suitable for riset-itb purpose
### riset-itb-block-wise
Code that suitable for riset-itb purpose
### riset-itb-napt
Code that suitable for riset-itb purpose

## v3.0.2 2021-07-29
### Added
- partitions.csv to do a partition for csv

### Changed
- how to send status that contains network discovery. The network discovery is disabled because it interrupts communication.
- Handle timeout using nack handler instead of timeout time
- Repair logging into using coap_log for coap client logging and set the level to COAP_LOG_NOTICE

### Removed
- ppm and pgm image format feature since it seems that the format is not relevant

### Fixed
- problem with jpeg
- repair zero-logging in total payload and count block
- remove unused variable

## v3.0.1 2021-07-26

### Fixed
- Tidying the code

### Added
- New esp32 camera component
