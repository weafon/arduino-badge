#include <Arduino.h>
#include <driver/i2s.h>

#include <BluetoothSerial.h>
BluetoothSerial BT;


const int BUTTON_PIN = GPIO_NUM_21;
const int LED_PIN = GPIO_NUM_19;

int buttonState;
int16_t buffer;  // Determine the Sampling time
int cnt = 0;

// you shouldn't need to change these settings

#define SAMPLE_RATE 16000
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_25
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_32
#define I2S_MIC_SERIAL_DATA GPIO_NUM_33



// don't mess around with this
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 1024,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};
//  .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S|I2S_COMM_FORMAT_I2S_LSB),
i2s_config_t i2sOut_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,  //8
  .dma_buf_len = 1024,
  .use_apll = 0,
  .tx_desc_auto_clear = true,
  .fixed_mclk = -1

};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};
#define I2S_SPK_LEFT_RIGHT_CLOCK GPIO_NUM_22
#define I2S_SPK_SERIAL_CLOCK GPIO_NUM_26
#define I2S_SPK_SERIAL_DATA GPIO_NUM_27

#define MAX98357_LRCL I2S_SPK_LEFT_RIGHT_CLOCK
#define MAX98357_BCLK I2S_SPK_SERIAL_CLOCK
#define MAX98357_DIN I2S_SPK_SERIAL_DATA
i2s_pin_config_t i2s_spk_pins = {
  .bck_io_num = I2S_SPK_SERIAL_CLOCK,
  .ws_io_num = I2S_SPK_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_SPK_SERIAL_DATA,
  .data_in_num = I2S_PIN_NO_CHANGE
};
void setup() {
  int i = 0;
  //delay(2000);
  //WiFi.mode(WIFI_OFF);
  Serial.begin(230400);
  delay(1000);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  i2s_driver_install(I2S_NUM_1, &i2sOut_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &i2s_spk_pins);
  i2s_set_sample_rates(I2S_NUM_1, SAMPLE_RATE);

  BT.begin("BTWEAFON");
  BT.begin(230400);
  Serial.println("");
  Serial.println("Started badge... 20240723");
  for (i = 0; i < 3; i++) {
	digitalWrite(LED_PIN, HIGH);
	delay(250);
	digitalWrite(LED_PIN, LOW);
	delay(250);
	Serial.println(3 - i);
  }
}
#define SAMPLE_BUFFER_SIZE 1024

#define SZ_HEADER_BT 2
#define SZ_PAYLOAD_BT (SAMPLE_BUFFER_SIZE*sizeof(int16_t))
#define SZ_BTFRAME (SZ_HEADER_BT+SZ_PAYLOAD_BT)
uint8_t buf_bt[SZ_PAYLOAD_BT + SZ_HEADER_BT];
#define SZ_PAYLOAD_PCM (SZ_PAYLOAD_BT*4)
uint8_t buf_pcm[SZ_PAYLOAD_PCM];
int off_rxframe = 0;
int off_pcmframe = 0;
int last_off = 0;
int cn_fail=0;
int cn_process=0;
int last_process =0;
int cn_miss=0;
unsigned long tm_start=0;
unsigned long tm=0;
unsigned long tm_curr, tm_frame;
void loop() 
{
	
	if (BT.available()>0) 
	{
		if (RecvBTPayload(buf_bt, SZ_BTFRAME)) 
		{
			process_btframe(buf_bt);
			cn_process++;				
		} else {
			abort_btframe();
			return;
		}
	} else {
		tm_curr = millis();
		if (((tm_curr-tm_start)>1000)&&(cn_process!=last_process))
		{
			Serial.printf("[%lu] off= %d got %d bad %d miss %d last_off %d\n", tm, off_rxframe, cn_process,cn_fail, cn_miss, last_off);
			tm_start=tm_curr;
			last_off=0;
			last_process = cn_process;
		}
	}
}

bool RecvBTPayload(uint8_t* pbuf, int sz)
{
	int i;
	unsigned long tm_wait = millis();
	for(i=0;i<sz;i++)
	{
		while(BT.available() <= 0) 
		{
			if ((millis()-tm_wait)>500) {
				off_rxframe = i;
				return false;
			}
		}
		pbuf[i] = BT.read();
		tm_wait = millis();
	}
	return true;
}

void abort_btframe()
{
	cn_miss++;
	cn_process++;
	BT.write(1);
}
void process_btframe(uint8_t* pbuf) 
{
	size_t cn_read=0;
	if (check_btframe(pbuf)==false)
	{
		cn_fail++;
		BT.write(1);
		return;
	}
	/*
	memcpy(buf_pcm+off_pcmframe, pbuf+SZ_HEADER_BT, SZ_PAYLOAD_BT);
	off_pcmframe+=SZ_PAYLOAD_BT;
	if (off_pcmframe==SZ_PAYLOAD_PCM) {
		i2s_write(I2S_NUM_1, (int16_t*)buf_pcm, SZ_PAYLOAD_PCM, &cn_read, portMAX_DELAY);
		off_pcmframe=0;
	}*/
	i2s_write(I2S_NUM_1, (int16_t*)(pbuf+SZ_HEADER_BT), SZ_PAYLOAD_BT, &cn_read, portMAX_DELAY);
	BT.write(0);
	return;
}

bool check_btframe(uint8_t* pbuf)
{
	int i;
	if (pbuf[0]!='W')
	{
		Serial.printf("got %c not W at header\n", pbuf[0]);
		return false;
	}
		
	return true;
	uint8_t chksum = 0;
	for(i=0;i<SZ_PAYLOAD_BT;i++) 
		chksum^=pbuf[i+SZ_HEADER_BT];
	if ((chksum^pbuf[1])==0)
		return true;
	else {
		Serial.printf("expect 0x%x real 0x%d\n", pbuf[1], chksum);
		return false;
	}
		
}




















