

extern "C"
{
	bool platformio_transport_open_serial(struct uxrCustomTransport *transport);
	bool platformio_transport_close_serial(struct uxrCustomTransport *transport);
	size_t platformio_transport_write_serial(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
	size_t platformio_transport_read_serial(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
}

// 【登记注册大门】
// 这个函数是你自己在程序里调用的。
// 作用是拿着上面那 4 个写好的函数，去 Micro-ROS 办事处“登记备案”。
static bool set_microros_serial_transports(Stream &stream)
{
rmw_uros_set_custom_transport(
        true,                               // true 表示这是流式通信（Stream），没有丢包概念
        &stream,                            // 把串口对象的地址交上去，以后就用它发数据
        platformio_transport_open_serial,   // 登记开门函数
        platformio_transport_close_serial,  // 登记关门函数
        platformio_transport_write_serial,  // 登记写函数
        platformio_transport_read_serial);  // 登记读函数
    return true;
}