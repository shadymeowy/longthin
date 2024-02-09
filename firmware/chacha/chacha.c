#include "chacha.h"

#define load32(x) (((uint32_t)(x)[0]) | ((uint32_t)(x)[1] << 8) | ((uint32_t)(x)[2] << 16) | ((uint32_t)(x)[3] << 24))
#define store32(x, v) \
    (x)[0] = (uint8_t)(v), (x)[1] = (uint8_t)((v) >> 8), (x)[2] = (uint8_t)((v) >> 16), (x)[3] = (uint8_t)((v) >> 24)

#define ROTL(a, b) (((a) << (b)) | ((a) >> (32 - (b))))
#define QR(a, b, c, d) (             \
    a += b, d ^= a, d = ROTL(d, 16), \
    c += d, b ^= c, b = ROTL(b, 12), \
    a += b, d ^= a, d = ROTL(d, 8),  \
    c += d, b ^= c, b = ROTL(b, 7))
#define ROUNDS 8 // This would be enough

void chacha_block(uint32_t out[16], uint32_t const in[16])
{
    int i;
    uint32_t x[16];

    for (i = 0; i < 16; ++i)
        x[i] = in[i];
    for (i = 0; i < ROUNDS; i += 2)
    {
        // Odd round
        QR(x[0], x[4], x[8], x[12]);  // column 0
        QR(x[1], x[5], x[9], x[13]);  // column 1
        QR(x[2], x[6], x[10], x[14]); // column 2
        QR(x[3], x[7], x[11], x[15]); // column 3
        // Even round
        QR(x[0], x[5], x[10], x[15]); // diagonal 1 (main diagonal)
        QR(x[1], x[6], x[11], x[12]); // diagonal 2
        QR(x[2], x[7], x[8], x[13]);  // diagonal 3
        QR(x[3], x[4], x[9], x[14]);  // diagonal 4
    }
    for (i = 0; i < 16; ++i)
        out[i] = x[i] + in[i];
}

void chacha_init(struct chacha_t *ctx, const uint8_t key[32])
{
    static const uint8_t sigma[16] = "expand 32-byte k";
    int i;
    for (i = 0; i < 4; ++i)
        ctx->input[i] = load32(sigma + i * 4);
    for (i = 4; i < 12; ++i)
        ctx->input[i] = load32(key + (i - 4) * 4);
    // load zeros for rest
    for (i = 12; i < 16; ++i)
        ctx->input[i] = 0;
}

void chacha_nonce_set8(struct chacha_t *ctx, const uint8_t nonce[8])
{
    ctx->input[14] = load32(nonce);
    ctx->input[15] = load32(nonce + 4);
}

void chacha_nonce_set32(struct chacha_t *ctx, uint32_t nonce[2])
{
    ctx->input[14] = nonce[0];
    ctx->input[15] = nonce[1];
}

void chacha_crypt(struct chacha_t *ctx, uint8_t *in_out, size_t len)
{
    size_t i;
    for (i = 0; i < len; i += 64)
    {
        chacha_block(ctx->output, ctx->input);
        // increment counter
        ctx->input[12]++;
        if (ctx->input[12] == 0)
        {
            ctx->input[13]++;
        }
        for (size_t j = 0; j < 64 && i + j < len; j++)
        {
            in_out[i + j] ^= ((uint8_t *)ctx->output)[j];
        }
    }
    ctx->input[12] = 0;
    ctx->input[13] = 0;
}