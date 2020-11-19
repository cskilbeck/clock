#pragma once

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/**
 * JSON type identifier. Basic types are:
 * 	o Object
 * 	o Array
 * 	o String
 * 	o Other primitive: number, boolean (true/false) or null
 */
typedef enum
{
    JSMN_UNDEFINED = 0,
    JSMN_OBJECT = 1,
    JSMN_ARRAY = 2,
    JSMN_STRING = 3,
    JSMN_PRIMITIVE = 4
} jsmntype_t;

enum jsmnerr
{
    /* Not enough tokens were provided */
    JSMN_ERROR_NOMEM = -1,
    /* Invalid character inside JSON string */
    JSMN_ERROR_INVAL = -2,
    /* The string is not a full JSON packet, more bytes expected */
    JSMN_ERROR_PART = -3
};

/**
 * JSON token description.
 * type		type (object, array, string etc.)
 * start	start position in JSON data string
 * end		end position in JSON data string
 */
typedef struct
{
    jsmntype_t type;
    int start;
    int end;
    int size;
#ifdef JSMN_PARENT_LINKS
    int parent;
#endif

#if defined(__cplusplus)

    char const *ptr(char const *json) const
    {
        return json + start;
    }

    int len() const
    {
        return end - start;
    }

    void get(char *dest, int dest_len, char const *json) const
    {
        int cp = len();
        if(cp >= dest_len) {
            cp = dest_len - 1;
        }
        memcpy(dest, json + start, cp);
        dest[cp] = 0;
    }

    void get_flt(float &result, char const *json) const
    {
        result = atof(json + start);
    }

    void get_int(int &result, char const *json) const
    {
        result = atoi(json + start);
    }

    template <typename T, size_t N> bool eq(T (&str)[N], char const *json) const
    {
        return type == JSMN_STRING && len() == (N - 1) && strncmp(json + start, str, N - 1) == 0;
    }

    template <typename T, size_t N> void get_str(T (&dest)[N], char const *json) const
    {
        get(dest, N, json);
    }

#endif
} jsmntok_t;

/**
 * JSON parser. Contains an array of token blocks available. Also stores
 * the string being parsed now and current position in that string
 */
typedef struct
{
    unsigned int pos;     /* offset in the JSON string */
    unsigned int toknext; /* next token to allocate */
    int toksuper;         /* superior token node, e.g parent object or array */
} jsmn_parser;

/**
 * Create JSON parser over an array of tokens
 */
void jsmn_init(jsmn_parser *parser);

/**
 * Run JSON parser. It parses a JSON data string into and array of tokens, each describing
 * a single JSON object.
 */
int jsmn_parse(jsmn_parser *parser, const char *js, size_t len, jsmntok_t *tokens, unsigned int num_tokens);
