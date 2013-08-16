# See the class UpdateRunner()
# status:           One-word description of the current action being performed
# long_description: Details pertaining to status if any.  Used for verbose error messages.

uint16  status
float32 progress
string  long_description

uint16 STS_IDLE            = 0
uint16 STS_INVALID         = 1
uint16 STS_BUSY            = 2
uint16 STS_CANCELLED       = 3
uint16 STS_ERR             = 4
uint16 STS_MOUNT_UPDATE    = 5
uint16 STS_VERIFY_UPDATE   = 6
uint16 STS_PREP_STAGING    = 7
uint16 STS_MOUNT_STAGING   = 8
uint16 STS_EXTRACT_UPDATE  = 9
uint16 STS_LOAD_KEXEC      = 10

