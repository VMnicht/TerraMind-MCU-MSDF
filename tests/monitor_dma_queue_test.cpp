#include "../TerraMind_FW/BSP/monitor_uart6_dma.h"

#include <assert.h>
#include <string>

static const uint8_t *g_dma_data = NULL;
static uint16_t g_dma_length = 0u;

extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,
                                                     uint8_t *data, uint16_t length)
{
    g_dma_data = data;
    g_dma_length = length;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *)
{
    return HAL_OK;
}

int main()
{
    UART_HandleTypeDef uart = {};
    uart.hdmatx = &uart;
    MonitorUart6Dma &tx = MonitorUart6Dma::instance();
    assert(tx.start(&uart));

    const std::string first(4000, 'A');
    assert(tx.enqueue(first.data(), first.size()));
    tx.service();
    assert(g_dma_length == 4000u);
    assert(std::string(reinterpret_cast<const char *>(g_dma_data), g_dma_length) == first);

    const std::string second(5000, 'B');
    assert(!tx.enqueue(second.data(), second.size())); // No overwrite during DMA.
    tx.on_tx_complete(&uart);
    assert(tx.enqueue(second.data(), second.size()));
    tx.service();
    assert(g_dma_length == 4192u); // First contiguous part before ring wrap.
    assert(std::string(reinterpret_cast<const char *>(g_dma_data), g_dma_length) ==
           second.substr(0, 4192));
    tx.on_tx_complete(&uart);
    // Completion immediately starts the wrapped remainder without a task tick.
    assert(g_dma_length == 808u);
    assert(std::string(reinterpret_cast<const char *>(g_dma_data), g_dma_length) ==
           second.substr(4192));
    tx.on_tx_complete(&uart);
    const MonitorTxStats stats = tx.stats();
    assert(stats.queue_bytes == 0u && !stats.dma_busy);
    assert(stats.records_queued == 2u && stats.records_dropped == 1u);
    assert(stats.bytes_sent == 9000u);
    assert(stats.queue_high_water == 5000u);

    assert(tx.enqueue("oops", 4u));
    tx.service();
    tx.on_uart_error(&uart);
    tx.service();
    const MonitorTxStats failed = tx.stats();
    assert(failed.queue_bytes == 0u && failed.bytes_discarded == 4u);
    assert(failed.uart_errors == 1u);
    return 0;
}
