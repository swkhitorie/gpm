__BEGIN_DECLS

int gpm_platform_init(void);

__END_DECLS

#ifdef __cplusplus

namespace gpm
{

/**
 * Startup init method. It has no specific functionality, just prints a welcome
 * message and sets the thread name
 */
__EXPORT void init(int argc, char *argv[], const char *process_name);

}

#endif
