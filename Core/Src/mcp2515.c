#include "../Inc/mcp2515.h"
#include "../Inc/gpio.h"

#define MCP_RESET       0xC0  // MCP2515 reset SPI 명령
#define MCP_READ        0x03  // MCP2515 register read SPI 명령
#define MCP_WRITE       0x02  // MCP2515 register write SPI 명령
#define MCP_BITMOD      0x05  // MCP2515 bit modify SPI 명령
#define MCP_RTS_TX0     0x81  // TX buffer 0 전송 요청 명령

#define MCP_CANSTAT     0x0E  // CAN 상태 레지스터
#define MCP_CANCTRL     0x0F  // CAN 제어 레지스터
#define MCP_CNF3        0x28  // CAN bit timing 설정 레지스터 3
#define MCP_CNF2        0x29  // CAN bit timing 설정 레지스터 2
#define MCP_CNF1        0x2A  // CAN bit timing 설정 레지스터 1
#define MCP_CANINTE     0x2B  // CAN interrupt enable 레지스터
#define MCP_CANINTF     0x2C  // CAN interrupt flag 레지스터

#define MCP_RXB0CTRL    0x60  // RX buffer 0 제어 레지스터
#define MCP_RXB0SIDH    0x61  // RX buffer 0 표준 ID 상위 레지스터
#define MCP_RXB0SIDL    0x62  // RX buffer 0 표준 ID 하위 레지스터
#define MCP_RXB0DLC     0x65  // RX buffer 0 데이터 길이 레지스터
#define MCP_RXB0D0      0x66  // RX buffer 0 첫 데이터 바이트
#define MCP_RXB1CTRL    0x70  // RX buffer 1 제어 레지스터
#define MCP_RXB1SIDH    0x71  // RX buffer 1 표준 ID 상위 레지스터
#define MCP_RXB1SIDL    0x72  // RX buffer 1 표준 ID 하위 레지스터
#define MCP_RXB1DLC     0x75  // RX buffer 1 데이터 길이 레지스터
#define MCP_RXB1D0      0x76  // RX buffer 1 첫 데이터 바이트

#define MCP_TXB0CTRL    0x30  // TX buffer 0 제어 레지스터
#define MCP_TXB0SIDH    0x31  // TX buffer 0 표준 ID 상위 레지스터
#define MCP_TXB0SIDL    0x32  // TX buffer 0 표준 ID 하위 레지스터
#define MCP_TXB0DLC     0x35  // TX buffer 0 데이터 길이 레지스터
#define MCP_TXB0D0      0x36  // TX buffer 0 첫 데이터 바이트

#define MCP_RX0IF       0x01  // RX buffer 0 수신 완료 flag
#define MCP_RX1IF       0x02  // RX buffer 1 수신 완료 flag
#define MCP_RX0IE       0x01  // RX buffer 0 interrupt enable bit
#define MCP_RX1IE       0x02  // RX buffer 1 interrupt enable bit
#define MCP_TXREQ       0x08  // TX buffer 전송 진행 bit

static void mcp_cs_low(void)  { GPIO_CLEAR_ODR(MCP_CS_PORT, MCP_CS_PIN); }  // MCP2515 SPI CS 선택
static void mcp_cs_high(void) { GPIO_SET_ODR(MCP_CS_PORT, MCP_CS_PIN); }    // MCP2515 SPI CS 해제

void spi2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;  // SPI2 클럭 활성화
    (void)RCC->APB1ENR;                  // 클럭 활성화 후 레지스터 반영 대기용 dummy read

    SPI2->CR1 = 0;  // SPI2 제어 레지스터 초기화
    SPI2->CR2 = 0;  // SPI2 보조 제어 레지스터 초기화
    SPI2->CR1 |= SPI_CR1_MSTR;  // master 모드
    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // software NSS 사용
    SPI2->CR1 |= SPI_CR1_BR_2;  // APB1 48MHz / 32 = 1.5MHz SPI clock
    SPI2->CR1 |= SPI_CR1_SPE;  // SPI2 활성화
}

static uint8_t spi2_transfer_8bit(uint8_t tx)
{
    while (!(SPI2->SR & SPI_SR_TXE)) {}  // 송신 버퍼가 빌 때까지 대기
    *((volatile uint8_t *)&SPI2->DR) = tx;  // 8비트 데이터 송신
    while (!(SPI2->SR & SPI_SR_RXNE)) {}  // 수신 데이터가 들어올 때까지 대기
    return *((volatile uint8_t *)&SPI2->DR);  // 동시에 수신된 8비트 데이터 반환
}

static void mcp2515_reset(void)
{
    mcp_cs_low();                  // SPI 트랜잭션 시작
    spi2_transfer_8bit(MCP_RESET); // reset 명령 전송
    mcp_cs_high();                 // SPI 트랜잭션 종료
    for (volatile int i = 0; i < 20000; i++) __NOP();  // reset 완료 대기
}

static void mcp2515_write(uint8_t addr, uint8_t data)
{
    mcp_cs_low();                   // SPI 트랜잭션 시작
    spi2_transfer_8bit(MCP_WRITE);  // write 명령 전송
    spi2_transfer_8bit(addr);       // 대상 레지스터 주소 전송
    spi2_transfer_8bit(data);       // 기록할 데이터 전송
    mcp_cs_high();                  // SPI 트랜잭션 종료
}

static uint8_t mcp2515_read(uint8_t addr)
{
    uint8_t data;                   // 읽어온 레지스터 값
    mcp_cs_low();                   // SPI 트랜잭션 시작
    spi2_transfer_8bit(MCP_READ);   // read 명령 전송
    spi2_transfer_8bit(addr);       // 대상 레지스터 주소 전송
    data = spi2_transfer_8bit(0x00);  // dummy byte 송신하며 데이터 수신
    mcp_cs_high();                  // SPI 트랜잭션 종료
    return data;                    // 레지스터 값 반환
}

static void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    mcp_cs_low();                    // SPI 트랜잭션 시작
    spi2_transfer_8bit(MCP_BITMOD);  // bit modify 명령 전송
    spi2_transfer_8bit(addr);        // 대상 레지스터 주소 전송
    spi2_transfer_8bit(mask);        // 변경할 비트 마스크 전송
    spi2_transfer_8bit(data);        // 적용할 비트 값 전송
    mcp_cs_high();                   // SPI 트랜잭션 종료
}

uint8_t mcp2515_init_500k(void)
{
    mcp2515_reset();                    // MCP2515 하드 리셋
    mcp2515_write(MCP_CANCTRL, 0x80);  // 설정 모드 진입
    mcp2515_write(MCP_CNF1, 0x00);     // 500kbps CAN bit timing 설정
    mcp2515_write(MCP_CNF2, 0xF0);     // 500kbps CAN bit timing 설정
    mcp2515_write(MCP_CNF3, 0x86);     // 500kbps CAN bit timing 설정
    mcp2515_write(MCP_RXB0CTRL, 0x60); // RXB0 모든 표준/확장 프레임 수신 허용
    mcp2515_write(MCP_RXB1CTRL, 0x60); // RXB1 모든 표준/확장 프레임 수신 허용
    mcp2515_write(MCP_CANINTE, MCP_RX0IE | MCP_RX1IE);  // RX0/RX1 수신 인터럽트 허용
    mcp2515_write(MCP_CANINTF, 0x00);  // 인터럽트 flag 초기화
    mcp2515_write(MCP_CANCTRL, 0x00);  // normal 모드 진입
    for (volatile int i = 0; i < 10000; i++) __NOP();  // 모드 전환 대기
    return ((mcp2515_read(MCP_CANSTAT) & 0xE0) == 0x00);  // normal 모드 진입 성공 여부
}

uint8_t mcp2515_send_std(uint16_t sid, const uint8_t *data, uint8_t len)
{
    if (len > 8) len = 8;  // CAN payload 최대 8바이트로 제한
    if (mcp2515_read(MCP_TXB0CTRL) & MCP_TXREQ) return 0;  // 이전 TX0 전송 중이면 실패

    mcp2515_write(MCP_TXB0SIDH, (uint8_t)(sid >> 3));  // 표준 ID 상위 8비트 설정
    mcp2515_write(MCP_TXB0SIDL, (uint8_t)((sid & 0x07) << 5));  // 표준 ID 하위 3비트 설정
    mcp2515_write(MCP_TXB0DLC, len & 0x0F);  // 데이터 길이 설정
    for (uint8_t i = 0; i < len; i++) {
        mcp2515_write((uint8_t)(MCP_TXB0D0 + i), data[i]);  // payload 바이트 기록
    }

    mcp_cs_low();                    // SPI 트랜잭션 시작
    spi2_transfer_8bit(MCP_RTS_TX0); // TX buffer 0 전송 요청
    mcp_cs_high();                   // SPI 트랜잭션 종료
    return 1;                        // 전송 요청 성공
}

static uint8_t mcp2515_read_rx_buffer(uint8_t buf, uint16_t *sid, uint8_t *data, uint8_t *len)
{
    uint8_t sidh_addr = (buf == 0) ? MCP_RXB0SIDH : MCP_RXB1SIDH;  // 선택한 RX buffer의 SIDH 주소
    uint8_t sidl_addr = (buf == 0) ? MCP_RXB0SIDL : MCP_RXB1SIDL;  // 선택한 RX buffer의 SIDL 주소
    uint8_t dlc_addr  = (buf == 0) ? MCP_RXB0DLC  : MCP_RXB1DLC;   // 선택한 RX buffer의 DLC 주소
    uint8_t data_addr = (buf == 0) ? MCP_RXB0D0   : MCP_RXB1D0;    // 선택한 RX buffer의 첫 데이터 주소
    uint8_t sidh = mcp2515_read(sidh_addr);  // 수신 표준 ID 상위 값
    uint8_t sidl = mcp2515_read(sidl_addr);  // 수신 표준 ID 하위 값
    uint8_t dlc = mcp2515_read(dlc_addr) & 0x0F;  // 수신 데이터 길이

    if (dlc > 8) dlc = 8;  // CAN payload 최대 길이 보정
    *sid = ((uint16_t)sidh << 3) | ((uint16_t)sidl >> 5);  // 표준 ID 복원
    *len = dlc;  // 데이터 길이 반환
    for (uint8_t i = 0; i < dlc; i++) {
        data[i] = mcp2515_read((uint8_t)(data_addr + i));  // payload 바이트 읽기
    }

    if (buf == 0) mcp2515_bit_modify(MCP_CANINTF, MCP_RX0IF, 0);  // RX0 수신 flag clear
    else mcp2515_bit_modify(MCP_CANINTF, MCP_RX1IF, 0);            // RX1 수신 flag clear
    return 1;  // 수신 프레임 읽기 성공
}

uint8_t mcp2515_receive(uint16_t *sid, uint8_t *data, uint8_t *len)
{
    uint8_t intf = mcp2515_read(MCP_CANINTF);  // MCP2515 interrupt flag 확인
    if (intf & MCP_RX0IF) return mcp2515_read_rx_buffer(0, sid, data, len);  // RX buffer 0 수신 프레임 처리
    if (intf & MCP_RX1IF) return mcp2515_read_rx_buffer(1, sid, data, len);  // RX buffer 1 수신 프레임 처리
    return 0;  // 수신 프레임 없음
}
