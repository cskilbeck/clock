#pragma once

//////////////////////////////////////////////////////////////////////
// accumulates the content into a buffer

struct http
{
    byte *data = null;
    size_t capacity = 0;
    size_t length = 0;

    int status_code;
    int content_length;

    void alloc(size_t max_size = 1024);
    void release();

    esp_err_t get(char const *url);
};
