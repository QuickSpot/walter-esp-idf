# Pass/fail criteria for tests\Run.ps1. Edit this, not the script.
#
# Marker syntax used by Whitelist / Blacklist / Pattern / ResetOn:
#   'literal text'  -> case-sensitive substring match
#   're:<regex>'    -> case-sensitive .NET regex match
#
# Every example logs under the same tag "[EXAMPLE]", so only the banner tells
# them apart. Match it WITHOUT the "(IDF v1.5.1)" suffix, so a release bump
# does not invalidate every criterion.

@{
    # ---------------------------------------------------------------------
    # Applied to every example on top of its own lists.
    # ---------------------------------------------------------------------
    Shared = @{

        # Milestones every example reaches on the happy path. Deliberately
        # short: coap says "Modem initialization OK" where the others say
        # "Successfully initialized the modem".
        Whitelist = @(
            'Created PDP context'
            'Successfully set operational state to FULL'
            'Connected to the network'
        )

        Blacklist = @(
            # --- ESP-IDF panic handler / watchdogs -----------------------
            'Guru Meditation'
            'abort() was called'
            'assert failed:'
            'Backtrace:'
            'CPU halted.'
            'Stack smashing'
            'Stack canary watchpoint'
            'Brownout detector was triggered'
            'Interrupt wdt timeout'
            'task_wdt: Task watchdog got triggered'
            'task_wdt: Aborting.'
            'Core dump started'
            # --- bootloader / image ---------------------------------------
            'has invalid magic byte'
            'Factory app partition is not bootable'
            # --- network registration refused (logged at INFO, not ERROR) --
            'Network registration: Denied'
            'Network registration state changed: Denied'
            # --- the modem driver's entire ESP_LOGE surface ---------------
            'No free buffers'
            'No free sockets available'
        )

        # A single AT retry is normal; three means the modem stopped
        # answering. This is the only "modem is dead" signal the driver has.
        RepeatLimits = @(
            @{ Pattern = 'Command time-out (TX) Attempt'; Max = 3; ResetOn = '' }
            @{ Pattern = 'Command time-out (WAIT)';       Max = 3; ResetOn = '' }
        )

        # At DEBUG the driver dumps every AT exchange, and those lines carry
        # literal "ERROR" text from the modem - never matched against.
        IgnoreLines = @(
            're:WalterModem: (RX|TX):'
        )

        # Reset causes that are a failure whatever the example.
        FatalResets = @(
            're:rst:0x7\b'      # TG0WDT_SYS_RST
            're:rst:0x8\b'      # TG1WDT_SYS_RST
            're:rst:0x9\b'      # RTCWDT_SYS_RST
            're:rst:0xb\b'      # TG1WDT_CPU_RST
            're:rst:0xf\b'      # BROWN_OUT_RST
            're:rst:0x10\b'     # RTCWDT_RTC_RST
        )
    }

    # ---------------------------------------------------------------------
    # Per example.
    #
    #   Banner        the line that identifies this example in the log.
    #   Features      WALTER_MODEM_ENABLE_* flags that must report
    #                 "<FEATURE> enabled: true" at configure time. Proves the
    #                 local checkout and the feature flags took.
    #   Infra         external services a pass depends on; printed next to a
    #                 failure, because a dead public broker looks like a bug.
    #   AllowedResets how many "rst:0x" banners are normal. 1 = the initial
    #                 boot; a second means an esp_restart() path ran.
    #   BenignResets  reset causes that are part of the example's design.
    #   TimeoutSec    wall-clock budget; 0 = use the run-wide value.
    #   QuietSec      silence before SILENT; 0 = use the run-wide value.
    # ---------------------------------------------------------------------
    Examples = @{

        bluecherry = @{
            Banner       = '=== Walter BlueCherry example'
            Features     = @('BLUECHERRY', 'SOCKETS')
            Infra        = 'BlueCherry tenant with device type "walter01" set to WAIT-PROVISION (coap.bluecherry.io:5684, ZTP :5688)'
            TimeoutSec   = 600
            QuietSec     = 240
            AllowedResets = 1
            # Ends every run with modem.sleep(5 min) -> deep sleep, so a
            # DSLEEP wake is the normal cycle boundary, not a crash.
            BenignResets = @('re:rst:0x5\b')
            Whitelist = @(
                'Successfully initialized the modem'
                'Successfully initialized BlueCherry'
                'Synchronized with the BlueCherry cloud platform'
            )
            Blacklist = @(
                'Could not initialize the modem'
                'Could not initialize BlueCherry'
                'Unable to connect to cellular network, restarting Walter'
                'Could not connect to BlueCherry server'
                'Could not sync with cloud'
                'Could not sync payload with cloud'
                'Could not sync internal event with cloud'
                'The BlueCherry socket was closed by the modem'
                'Could not write to the BlueCherry cloud connection'
                'No modem socket is available for BlueCherry'
                'Could not configure the BlueCherry socket'
                'Could not enable DTLS on the BlueCherry socket'
                'Could not configure device credentials'
                '(ZTP) Provisioning failed'
                '(ZTP) This device might not exist'
                '(ZTP) Could not connect to the provisioning server'
                '(ZTP) Could not identify this device to the provisioning server'
                're:OTA: firmware v\d+ failed with error'
            )
            RepeatLimits = @()
        }

        coap = @{
            Banner       = '=== WalterModem CoAP example'
            Features     = @('COAP')
            Infra        = 'coap.me:5683 (CoAP over UDP)'
            TimeoutSec   = 0
            QuietSec     = 0
            AllowedResets = 1
            BenignResets = @()
            # coap does call coapCreateContext, so "CoAP: Connected
            # successfully" can fire - but it rides a +SQNCOAPCONNECTED URC
            # and the RING line below proves strictly more, so it is left out.
            Whitelist = @(
                'Modem initialization OK'
                'Successfully created or refreshed CoAP context'
                'Sent CoAP datagram'
                'CoAP: Message received on profile 1.'
                're:Received (empty message|message) for profile 1'
            )
            Blacklist = @(
                'Modem initialization ERROR'
                'Failed to register to network'
                'Could not create CoAP context.'
                'Could not set CoAP header'
                'Could not send CoAP datagram'
                'Could not receive CoAP message'
            )
            RepeatLimits = @()
        }

        http = @{
            Banner       = '=== WalterModem HTTP example'
            Features     = @('HTTP')
            Infra        = 'quickspot.io:80 (/hello/get, /hello/post)'
            TimeoutSec   = 0
            QuietSec     = 0
            AllowedResets = 1
            BenignResets = @()
            # There is deliberately no "HTTP: Connected successfully" marker:
            # that event rides a +SQNHTTPCONNECT URC, and neither http nor
            # https ever calls httpConnect() - both go straight to
            # httpQuery/httpSend and let the modem connect implicitly. Waiting
            # for it just burns the whole timeout.
            Whitelist = @(
                'Successfully initialized the modem'
                'Successfully configured the HTTP profile'
                'HTTP GET successfully sent'
                're:HTTP: Message received on profile 1\. \(status: 200'
                'Received message for profile 1:'
                'HTTP POST successfully sent'
            )
            Blacklist = @(
                'Could not initialize the modem'
                'Failed to configure HTTP profile'
                'Failed to register to network'
                'HTTP GET query failed'
                'HTTP POST failed'
                'HTTP: Connection (profile 1) could not be established.'
                'HTTP: Connection (profile 1) was interrupted'
                'Could not receive HTTP message for profile'
            )
            RepeatLimits = @()
        }

        https = @{
            Banner       = '=== WalterModem HTTPS example'
            Features     = @('HTTP')
            Infra        = 'quickspot.io:443, ISRG Root X1 pinned in main/https.cpp'
            TimeoutSec   = 0
            QuietSec     = 0
            AllowedResets = 1
            BenignResets = @()
            # Its event handler prints "HTTP:", not "HTTPS:" - only the banner
            # and the "HTTPS GET/POST" lines distinguish it from the http run.
            # See http above for why there is no "Connected successfully".
            Whitelist = @(
                'Successfully initialized the modem'
                'TLS profile configured'
                'TLS Profile setup succeeded'
                'Successfully configured the HTTP profile'
                'HTTPS GET successfully sent'
                're:HTTP: Message received on profile 1\. \(status: 200'
                'Received message for profile 1:'
                'HTTPS POST successfully sent'
            )
            Blacklist = @(
                'Could not initialize the modem'
                'CA cert upload failed'
                'TLS profile configuration failed'
                'TLS Profile setup failed'
                'Failed to configure HTTP profile'
                'Failed to register to network'
                'HTTPS GET query failed'
                'HTTPS POST failed'
                'HTTP: Connection (profile 1) could not be established.'
                'HTTP: Connection (profile 1) was interrupted'
                'Could not receive HTTP message for profile'
            )
            RepeatLimits = @()
        }

        mqtt = @{
            Banner       = '=== WalterModem MQTT example'
            Features     = @('MQTT')
            Infra        = 'broker.emqx.io:1883, public topic walter-test-topic'
            TimeoutSec   = 0
            QuietSec     = 0
            AllowedResets = 1
            BenignResets = @()
            Whitelist = @(
                'Successfully initialized the modem'
                'Successfully configured the MQTT client'
                'MQTT: Connected successfully'
                "MQTT: Successfully subscribed to topic 'walter-test-topic'"
                're:MQTT: Successfully published message \(id: \d+\)'
                're:MQTT: Message \(id: \d+\) received on topic ''walter-test-topic'''
                'Received message: walter-'
            )
            # Nine of this example's failure strings are logged at INFO, not
            # ERROR. Match the text; never the level.
            Blacklist = @(
                'Could not initialize the modem'
                'Failed to configure MQTT client'
                'Failed to connect to network'
                'MQTT: Connection could not be established.'
                'MQTT: Could not subscribe to topic.'
                'MQTT: Connection was interrupted'
                'MQTT: Could not publish message'
                'Subscribing failed'
                'Publishing failed'
                'MQTT publish failed'
                'Could not receive MQTT message'
                'MQTT: Memory full'
            )
            # A refused broker makes the example re-dial every 5s forever with
            # no error and no restart - nothing else would ever catch it.
            RepeatLimits = @(
                @{ Pattern = 'Connecting to MQTT broker...'; Max = 4; ResetOn = 'MQTT: Connected successfully' }
            )
        }

        mqtts = @{
            Banner       = '=== WalterModem MQTTS example'
            Features     = @('MQTT')
            Infra        = 'broker.emqx.io:8883 (TLS), DigiCert Global Root CA pinned in main/mqtts.cpp, public topic walter-tls-test-topic'
            TimeoutSec   = 0
            QuietSec     = 0
            AllowedResets = 1
            BenignResets = @()
            Whitelist = @(
                'Successfully initialized the modem'
                'TLS profile configured'
                'TLS Profile setup succeeded'
                'Successfully configured the MQTT client'
                'MQTT: Connected successfully'
                "MQTT: Successfully subscribed to topic 'walter-tls-test-topic'"
                're:MQTT: Successfully published message \(id: \d+\)'
                're:MQTT: Message \(id: \d+\) received on topic ''walter-tls-test-topic'''
                'Received message: walter-'
            )
            Blacklist = @(
                'Could not initialize the modem'
                'CA cert upload failed'
                'TLS profile configuration failed'
                'TLS Profile setup failed'
                'Failed to configure MQTT client'
                'Failed to connect to network'
                'MQTT: Connection could not be established.'
                'MQTT: Could not subscribe to topic.'
                'MQTT: Connection was interrupted'
                'MQTT: Could not publish message'
                'Subscribing failed'
                'Publishing failed'
                'MQTT publish failed'
                'Could not receive MQTT message'
                'MQTT: Memory full'
            )
            RepeatLimits = @(
                @{ Pattern = 'Connecting to MQTT broker...'; Max = 4; ResetOn = 'MQTT: Connected successfully' }
            )
        }

        positioning = @{
            Banner       = '=== WalterModem Positioning example'
            Features     = @('GNSS', 'SOCKETS')
            Infra        = 'walterdemo.quickspot.io:1999 (UDP), a SIM with data for GNSS assistance, and a GNSS antenna with sky view'
            # First cycle = LTE attach (up to 300s) + NITZ + almanac and
            # ephemeris download + the fix itself + a second full attach.
            TimeoutSec   = 900
            QuietSec     = 300
            AllowedResets = 1
            BenignResets = @()
            Whitelist = @(
                'Successfully initialized the modem'
                'Successfully configured a new socket'
                'Successfully set socket to insecure mode'
                're:Started GNSS fix \(attempt \d+/\d+\)'
                'GNSS fix received: Confidence:'
                'Successfully obtained a valid GNSS fix'
                'Successfully dialed demo server'
            )
            # No "sent OK" line exists after socketSend - success there is the
            # absence of "Could not transmit data".
            Blacklist = @(
                'Could not initialize the modem'
                'Could not configure the GNSS subsystem'
                'Could not configure a new socket'
                'Could not disable socket TLS'
                'Could not connect to the LTE network'
                'Could not validate GNSS clock'
                'Could not request GNSS fix'
                'Could not dial demo server'
                'Could not transmit data'
                'Could not close the socket'
            )
            # attemptGNSSFix gives up after 5 tries and logs nothing on the
            # way out - the run would otherwise look alive but never pass.
            RepeatLimits = @(
                @{ Pattern = 'GNSS fix confidence'; Max = 5; ResetOn = 'Successfully obtained a valid GNSS fix' }
            )
        }

        tcp = @{
            Banner       = '=== WalterModem TCP example'
            Features     = @('SOCKETS')
            Infra        = 'walterdemo.quickspot.io:1999 (TCP listener; a real 3-way handshake, so a dial failure is meaningful)'
            TimeoutSec   = 0
            QuietSec     = 0
            AllowedResets = 1
            BenignResets = @()
            Whitelist = @(
                'Successfully initialized the modem'
                'Successfully configured a new socket'
                'Successfully set socket to insecure mode'
                're:Successfully connected Socket \d+ to TCP server'
                'Sending packet...'
                'TCP send basic packet succeeded'
            )
            Blacklist = @(
                'Could not initialize the modem'
                'Could not configure a new socket'
                'Could not disable socket TLS'
                'Failed to connect to network'
                'Could not dial TCP server'
                'TCP send packet failed'
            )
            RepeatLimits = @()
        }

        udp = @{
            Banner       = '=== WalterModem UDP example'
            Features     = @('SOCKETS')
            Infra        = 'walterdemo.quickspot.io:1999 (UDP). Connectionless: dial and send succeed even with nothing listening, so a pass proves the LTE path, not server reachability.'
            TimeoutSec   = 0
            QuietSec     = 0
            AllowedResets = 1
            BenignResets = @()
            Whitelist = @(
                'Successfully initialized the modem'
                'Successfully configured a new socket'
                'Successfully set socket to insecure mode'
                're:Successfully connected Socket \d+ to UDP server'
                'Sending packet...'
                'UDP send basic packet succeeded'
            )
            Blacklist = @(
                'Could not initialize the modem'
                'Could not configure a new socket'
                'Could not disable socket TLS'
                'Failed to connect to network'
                'Could not dial UDP server'
                'UDP send packet failed'
            )
            RepeatLimits = @()
        }
    }
}
