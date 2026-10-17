# Application-level operator liveness lease. It must be published for the
# lifetime of the control client, not only while a particular page is visible.
string session_id
string client_id
bool enabled
builtin_interfaces/Time stamp
