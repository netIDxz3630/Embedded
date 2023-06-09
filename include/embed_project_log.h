#ifndef EMBED_PROJECT_LOG
#define EMBED_PROJECT_LOG

#ifdef __cplusplus
extern "C" {
#endif

#ifdef EMBED_DEBUG 
#define EMBED_PROJECT_LOG(fmt, args...) \
{\
    printf("[Time:%s %s] File:%s Line:%d Function:%s, ", __DATE__, __TIME__, __FILE__, __LINE__, __FUNCTION__);\
    printf(fmt, ##args);\
}
#else
#define EMBED_PROJECT_LOG(fmt, args...)
#endif

#ifdef __cplusplus
}
#endif
#endif