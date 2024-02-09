#ifndef CHACHA
#define CHACHA

#if __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

struct chacha_t {
    uint32_t input[16];
    uint32_t output[16];
};

void chacha_init(struct chacha_t *ctx, const uint8_t key[32]);
void chacha_block(uint32_t out[16], uint32_t const in[16]);
void chacha_crypt(struct chacha_t *ctx, uint8_t *in_out, size_t len);
void chacha_nonce_set8(struct chacha_t *ctx, const uint8_t nonce[8]);
void chacha_nonce_set32(struct chacha_t *ctx, uint32_t nonce[2]);
void chacha_nonce_rotate(struct chacha_t *ctx);

#if __cplusplus
}
#endif

#endif
