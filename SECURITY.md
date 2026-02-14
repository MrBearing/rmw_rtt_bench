# Security Considerations

## Payload Size Limits

This benchmark suite implements security measures to prevent Denial of Service (DoS) attacks:

### Maximum Payload Size

- **Hard limit:** 10 MB (10,485,760 bytes)
- **Applies to:** Both pinger and ponger nodes
- **Rationale:** Prevents malicious or misconfigured nodes from exhausting system memory

### Pinger Validation

The pinger validates payload size at argument parsing:
- Rejects negative values
- Rejects values exceeding 10 MB
- Provides clear error messages

### Ponger Protection

The ponger validates incoming message payloads:
- Checks `payload_size_bytes` field from network messages
- Rejects messages with payload > 10 MB
- Logs warnings for rejected messages
- Prevents memory allocation attacks

## CSV Field Escaping

CSV output properly escapes special characters in user-provided fields:
- `transport_tag` and `notes` fields are escaped
- Handles commas, quotes, and newlines
- Prevents CSV injection and parsing errors

## Best Practices

When deploying this benchmark:

1. **Use firewalls** to restrict access to benchmark nodes
2. **Monitor resource usage** during extended runs
3. **Set appropriate QoS settings** for your network
4. **Validate input parameters** before launching
5. **Review logs** for rejected messages or anomalies

## Reporting Security Issues

If you discover a security vulnerability, please email the maintainer rather than opening a public issue.
