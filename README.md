
The trendnet_streamer binary expects identification information to be included
in a file src/trendnet_config.h which is not stored in Git (for obvious
reasons).   The file can include the following #defines:

    #define TN_USERNAME "username"      // Login username for trendnet_streamer
    #define TN_PASSWD   "password"      // Login password for trendnet_streamer

Anywhere these defines are used in the code they should be guarded by an #ifndef
to a default value.   That is, the compiler will complain if config.h doesn't
exist, but an empty file will suffice.
