#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

extern int rcar_canfd_send(const struct device *dev, int ch, uint32_t id, const uint8_t *data, uint8_t len);
extern int rcar_canfd_poll_recv(const struct device *dev, int ch, uint32_t *id, uint8_t *len, uint8_t *data);

int main(void)
{
    printk("Hello from can_loopback sample\n");
    const struct device *canfd = DEVICE_DT_GET(DT_NODELABEL(canfd));
    if (!device_is_ready(canfd)) {
        printk("canfd not ready\n");
        return 0;
    }

    uint8_t tx[8] = {0};
    uint8_t rx[64];
    uint32_t id;
    uint8_t len;
    uint32_t cnt = 1;
	volatile uint32_t dummy;

	tx[0] = (uint8_t)cnt;
	rcar_canfd_poll_recv(canfd, 4, &id, &len, rx);
	for (int i = 0; i<100000000; ++i) dummy=i;

	rcar_canfd_send(canfd, 3, 0x123, tx, 8);

	for (int i = 0; i<100000000; ++i) dummy=i;
	for (int i = 0; i<100000000; ++i) dummy=i;
	for (int i = 0; i<100000000; ++i) dummy=i;
	rcar_canfd_poll_recv(canfd, 4, &id, &len, rx);
	return 0;

	while(1) {
		while (!rcar_canfd_poll_recv(canfd, 4, &id, &len, rx)) {
			printk("RX(ch4) id=0x%x len=%u data=", id, len);
			for (int i=0;i<len;i++) printk("%02x ", rx[i]);
			printk("\n");
		}
	
		cnt++;
		//k_sleep(K_MSEC(1000));
		for (int i = 0; i<100000000; ++i) dummy=i;
	}
}
