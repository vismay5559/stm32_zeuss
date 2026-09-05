#ifndef DMA_BUFFER_H
#define DMA_BUFFER_H

/*
 * Where a buffer that hardware writes into has to live.
 *
 * Several peripherals here - USB, SPI, UART - fill their buffers themselves,
 * without the processor's involvement. The processor keeps a fast local copy
 * of recently used memory, and it has no way of knowing a peripheral has
 * changed the original underneath it. So it reads its own stale copy, and the
 * data goes intermittently wrong in a way that looks exactly like bad wiring.
 *
 * The fix is to put these buffers in a region the processor is told not to
 * keep copies of. Two things have to agree for that: the placement below, and
 * MPU Region 2 in the Boot project. app.c checks that agreement at startup
 * rather than trusting it, because CubeMX will happily regenerate the region
 * back to its default.
 *
 * The alignment matters separately: a cache line is 32 bytes, so a buffer
 * that starts mid-line shares that line with whatever is next to it.
 *
 * On a workstation there is no such region and no such processor. A host
 * compiler rejects the name outright, so the host build keeps the alignment
 * and drops only the placement - which means the buffers are laid out the
 * same way, and the part that is dropped is the part that only means anything
 * on the board. The target branch is unchanged.
 */
#ifdef NEXUS_HOSTTEST
#define NEXUS_DMA_BUFFER  __attribute__((aligned(32)))
#else
#define NEXUS_DMA_BUFFER  __attribute__((section("noncacheable_buffer"), aligned(32)))
#endif

#endif /* DMA_BUFFER_H */
