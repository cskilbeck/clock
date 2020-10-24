#pragma once

namespace json
{

    inline int len(jsmntok_t tok)
    {
        return tok.end - tok.start;
    }

    inline void get(jsmntok_t tok, char *dest, int dest_len, char const *json_text)
    {
        int cp = len(tok);
        if(cp >= dest_len) {
            cp = dest_len - 1;
        }
        memcpy(dest, json_text + tok.start, cp);
        dest[cp] = 0;
    }

    inline bool get_flt(jsmntok_t tok, float &result, char const *json_text)
    {
        if(tok.type == JSMN_PRIMITIVE) {
            result = (float)atof(json_text + tok.start);
            return true;
        }
        return false;
    }

    inline bool get_bool(jsmntok_t tok, bool &result, char const *json_text)
    {
        if(tok.type == JSMN_PRIMITIVE) {
            result = len(tok) == 4 && strncmp(json_text + tok.start, "true", 4) == 0;
            if(!result && (len(tok) != 5 || strncmp(json_text + tok.start, "false", 5) != 0)) {
                return false;
            }
            return true;
        }
        return false;
    }

    inline bool get_int(jsmntok_t tok, int &result, char const *json_text)
    {
        if(tok.type == JSMN_PRIMITIVE) {
            result = atoi(json_text + tok.start);
            return true;
        }
        return false;
    }

    inline bool get_uint32(jsmntok_t tok, unsigned int &result, char const *json_text)
    {
        if(tok.type == JSMN_PRIMITIVE) {
            char *end;
            result = strtoul(json_text + tok.start, &end, 10);
            return true;
        }
        return false;
    }

    template <typename T, size_t N> bool eq(jsmntok_t tok, T (&str)[N], char const *json_text)
    {
        return tok.type == JSMN_STRING && len(tok) == (N - 1) && strncmp(json_text + tok.start, str, N - 1) == 0;
    }

    template <typename T, size_t N> void get_str(jsmntok_t tok, T (&dest)[N], char const *json_text)
    {
        get(tok, dest, N, json_text);
    }
}    // namespace json
