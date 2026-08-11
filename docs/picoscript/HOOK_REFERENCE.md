# PicoScript Hook Reference (587 hooks, 76 namespaces)

Complete reference for all host hooks in the PicoScript 16-opcode ISA.
Each hook is a deterministic primitive callable from any of the 7 language surfaces.

## Summary

| Metric | Value |
|--------|-------|
| Total hooks | 587 |
| Namespaces | 76 |
| Language surfaces | 7 (C, BASIC, Python, English, COBOL, Report, Functional) |
| Execution paths | 5 (Python VM, JS VM, C VM, native C, native JS) |

---

## Core ISA

### Thread.* (1 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Thread.YieldCounted() | 0x0070 | Deterministic cooperative-yield counter; increments and returns a per-VM sequence number. Real on all 3 runtimes. |

### Net.* (10 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Net.Listen() | 0x02E0 | Host-backed socket listener. |
| Net.Accept() | 0x02E1 | Accept a connection from a listener. |
| Net.Read() | 0x02E2 | Receive bytes as a span. |
| Net.Write() | 0x02E3 | Send a span. |
| Net.Shutdown() | 0x02E4 | Close a listener or connection. |
| Net.PoolSize() | 0x02E5 | Configure a host worker pool. |
| Net.Register() | 0x02E6 | Register an event handler with the host. |
| Net.Connect() | 0x037E | Connect a client socket to endpoint + port. |
| Net.SendSpan() | 0x037F | Span-oriented send alias. |
| Net.RecvSpan() | 0x0380 | Span-oriented receive alias. |

---

## Memory & Spans

### Memory.* (9 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Memory.ArenaInit() | 0x0030 | |
| Memory.ArenaAlloc() | 0x0031 | |
| Memory.ArenaReset() | 0x0032 | |
| Memory.ArenaStats() | 0x0033 | |
| Memory.Peek() | 0x0034 | |
| Memory.Poke() | 0x0035 | |
| Memory.Set() | 0x0036 | |
| Memory.Get() | 0x0037 | |
| Memory.SetConst() | 0x005F | |

### Span.* (5 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Span.Make() | 0x0040 | |
| Span.Slice() | 0x0041 | |
| Span.Materialize() | 0x0042 | |
| Span.Len() | 0x0043 | |
| Span.Get() | 0x0044 | |

### Descriptor.* (6 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Descriptor.Make() | 0x0050 | Creates a pure buffer descriptor (ptr, len; flags=0) and returns a handle. No host state -- real, deterministic on all 3 runtimes. |
| Descriptor.SetFlags() | 0x0051 | Sets the driver-facing flags word on an existing descriptor (2-call ABI pattern, like String.SetReplace). |
| Descriptor.GetPtr() | 0x0052 | Returns the descriptor's arena pointer. |
| Descriptor.GetLen() | 0x0053 | Returns the descriptor's length. |
| Descriptor.GetFlags() | 0x0054 | Returns the descriptor's flags word. |
| Descriptor.CopyBatch() | 0x0055 | memcpy's min(src.len, dst.len) bytes from one descriptor's buffer to another's (same convention as Span.Materialize). |

### Arena.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Arena.Mark() | 0x007C | |
| Arena.Rewind() | 0x007D | |
| Arena.Reset() | 0x007E | |

### Lease.* (6 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Lease.Acquire() | 0x0058 | Creates a generic capability/ownership token over a span + type hint; returns a handle. No host state -- real, deterministic. Distinct from Stream.Next's own unrelated internal per-frame lease concept. |
| Lease.Release() | 0x0059 | Marks a lease token invalid. |
| Lease.Validate() | 0x005A | Returns 1 if the lease handle is valid (acquired and not released), else 0. |
| Lease.CachedValidate() | 0x005B | A host-optimization hint (a real host may memoize the check); the reference VM has no cache to distinguish, so it gives the same answer as Validate. |
| Lease.GetSpan() | 0x005C | Returns the span associated with a valid lease (empty span if invalid). |
| Lease.GetTypeHint() | 0x005D | Returns the type hint associated with a valid lease (0 if invalid). |

### Dot8.* (2 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Dot8.Len() | 0x0056 | |
| Dot8.Of() | 0x0057 | |

---

## I/O & Text

### Io.* (2 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Io.Write() | 0x0071 | |
| Io.WriteByte() | 0x0072 | |

### Utf8Writer.* (7 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Utf8Writer.New() | 0x0021 | |
| Utf8Writer.Byte() | 0x0022 | |
| Utf8Writer.Int() | 0x0023 | |
| Utf8Writer.Span() | 0x0024 | |
| Utf8Writer.ToSpan() | 0x0025 | |
| Utf8Writer.Len() | 0x0026 | |
| Utf8Writer.Reset() | 0x0027 | |

### Utf8Reader.* (8 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Utf8Reader.New() | 0x0028 | |
| Utf8Reader.Peek() | 0x0029 | |
| Utf8Reader.Next() | 0x002A | |
| Utf8Reader.Int() | 0x002B | |
| Utf8Reader.SkipWs() | 0x002C | |
| Utf8Reader.Eof() | 0x002D | |
| Utf8Reader.Pos() | 0x002E | |
| Utf8Reader.Match() | 0x002F | |

### Json.* (10 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Json.BeginObject() | 0x0045 | |
| Json.EndObject() | 0x0046 | |
| Json.BeginArray() | 0x0047 | |
| Json.EndArray() | 0x0048 | |
| Json.Key() | 0x0049 | |
| Json.Str() | 0x004A | |
| Json.Int() | 0x004B | |
| Json.Bool() | 0x004C | |
| Json.Null() | 0x004D | |
| Json.Raw() | 0x004E | |

### Xml.* (7 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Xml.Open() | 0x0073 | |
| Xml.AttrName() | 0x0074 | |
| Xml.AttrValue() | 0x0075 | |
| Xml.OpenEnd() | 0x0076 | |
| Xml.Text() | 0x0077 | |
| Xml.Close() | 0x0078 | |
| Xml.Empty() | 0x0079 | |

### TextRender.* (9 hooks)

| Method | Code | Description |
|--------|------|-------------|
| TextRender.Raw() | 0x0260 | |
| TextRender.Text() | 0x0261 | |
| TextRender.Open() | 0x0262 | |
| TextRender.Attr() | 0x0263 | |
| TextRender.OpenEnd() | 0x0264 | |
| TextRender.Close() | 0x0265 | |
| TextRender.Empty() | 0x0266 | |
| TextRender.Hole() | 0x0267 | |
| TextRender.Br() | 0x0268 | |

### Template.* (2 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Template.Compile() | 0x007A | |
| Template.Render() | 0x007B | |

---

## Storage & Query

### Storage.* (21 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Storage.GetSchemaForPack() | 0x0060 | |
| Storage.SetSchemaForPack() | 0x0061 | |
| Storage.AddCard() | 0x0062 | |
| Storage.UpdateCard() | 0x0063 | |
| Storage.DeleteCard() | 0x0064 | |
| Storage.PatchCard() | 0x0065 | |
| Storage.ReadCard() | 0x0066 | |
| Storage.QueryCard() | 0x0067 | |
| Storage.UsePack() | 0x0068 | |
| Storage.EditCard() | 0x0069 | |
| Storage.GetField() | 0x006A | |
| Storage.SetField() | 0x006B | |
| Storage.SetFieldStr() | 0x006C | |
| Storage.GetFieldStr() | 0x006D | |
| Storage.QueryResult() | 0x006E | |
| Storage.Ready() | 0x006F | |
| Storage.SetSlice() | 0x01A0 | |
| Storage.CardLen() | 0x01A1 | |
| Storage.ReadSlice() | 0x01A2 | |
| Storage.WriteSlice() | 0x01A3 | |
| Storage.IsUserPack() | 0x01A4 | |

### Query.* (2 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Query.BuildLookupFilter() | 0x01C0 | |
| Query.BuildManyToManyMap() | 0x01C1 | |

### Data.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Data.Lookup() | 0x0300 | Host-bound read: no active server/data context in the reference VM, so returns 0 -- a defined default, identical on all 3 runtimes, until a host injects a real data binding. |
| Data.FieldNum() | 0x0301 | Same: returns 0. |
| Data.FieldStr() | 0x0302 | Same: returns an empty span. |

### Search.* (28 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Search.Clear() | 0x01D0 | |
| Search.UpsertText() | 0x01D1 | |
| Search.Delete() | 0x01D2 | |
| Search.IndexPack() | 0x01D3 | |
| Search.QueryText() | 0x01D4 | |
| Search.SetVector() | 0x01D5 | |
| Search.QueryHybrid() | 0x01D6 | |
| Search.Result() | 0x01D7 | |
| Search.Score() | 0x01D8 | |
| Search.Plan() | 0x01D9 | |
| Search.SetSemanticWeight() | 0x01DA | |
| Search.Configure() | 0x01DB | |
| Search.Compatible() | 0x01DC | |
| Search.Rebuild() | 0x01DD | |
| Search.SetFacet() | 0x01DE | |
| Search.SetNumber() | 0x01DF | |
| Search.ClearFields() | 0x0200 | |
| Search.Facets() | 0x0201 | |
| Search.FacetValue() | 0x0202 | |
| Search.FacetCount() | 0x0203 | |
| Search.Range() | 0x0204 | |
| Search.Save() | 0x0205 | |
| Search.Load() | 0x0206 | |
| Search.JournalUpsert() | 0x0207 | |
| Search.JournalDelete() | 0x0208 | |
| Search.JournalFacet() | 0x0209 | |
| Search.JournalNumber() | 0x020A | |
| Search.JournalReplay() | 0x020B | |

---

## HTTP & Request

### Req.* (13 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Req.Seq() | 0x0007 | |
| Req.Principal() | 0x0008 | |
| Req.Method() | 0x0009 | |
| Req.Path() | 0x000A | |
| Req.Header() | 0x000B | |
| Req.BodyMode() | 0x000C | |
| Req.BodyCount() | 0x000D | |
| Req.BodySpan() | 0x000E | |
| Req.SetSlice() | 0x01B0 | |
| Req.BodySlice() | 0x01B1 | |
| Req.BodyLen() | 0x01B2 | |
| Req.Param() | 0x01B6 | |
| Req.ParamCount() | 0x01B7 | |

### Resp.* (13 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Resp.Status() | 0x0015 | |
| Resp.Header() | 0x0016 | |
| Resp.Write() | 0x0017 | |
| Resp.Trailer() | 0x0018 | |
| Resp.Seal() | 0x0019 | |
| Resp.End() | 0x001A | |
| Resp.Respond() | 0x001B | |
| Resp.Flush() | 0x001C | |
| Resp.Continue() | 0x001D | |
| Resp.EndStream() | 0x001E | |
| Resp.Upgrade() | 0x001F | |
| Resp.Abort() | 0x0038 | |
| Resp.EarlyHints() | 0x0039 | |

### Http.* (12 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Http.ReadHeader() | 0x0130 | Reads a header from the live host connection -- host-injected by design. Returns an empty span until a host installs a real binding. |
| Http.ReadBody() | 0x0131 | Reads the live connection's body -- host-injected. Returns an empty span. |
| Http.GenerateHeaders() | 0x0132 | Generates response headers for the live connection -- host-injected. Returns an empty span. |
| Http.GenerateResponse() | 0x0133 | Generates a full response for the live connection -- host-injected. Returns an empty span. |
| Http.ParseQuery() | 0x0134 | Pure: URL-decodes a query string into the Template `key=value` model. Implemented on all 3 runtimes. |
| Http.ParseForm() | 0x0135 | Pure: same as ParseQuery for form-encoded bodies. Implemented on all 3 runtimes. |
| Http.ParseJson() | 0x0136 | Pure: JSON -> dotted-path Template model (feeds `{{#each}}`). Implemented on all 3 runtimes. |
| Http.EncodeJson() | 0x0137 | Pure: Template model -> JSON object with escaping. Implemented on all 3 runtimes. |
| Http.Request() | 0x0138 | Makes an outbound HTTP request -- host-injected (live network). Returns 0 until a host installs a real binding. (Previously an undocumented gap -- silently fell through; now an explicit default.) |
| Http.RespStatus() | 0x0139 | Sets/reads a live response's status -- host-injected. Returns 0. (Previously undocumented -- now an explicit default.) |
| Http.RespHeaders() | 0x013A | Live response headers -- host-injected. Returns an empty span. (Previously undocumented -- now an explicit default.) |
| Http.RespBody() | 0x013B | Live response body -- host-injected. Returns an empty span. (Previously undocumented -- now an explicit default.) |

### Html.* (10 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Html.CreateNode() | 0x0140 | DOM tree ops are not built (a full mutable tree model is a separate, larger feature). Returns 0 (a defined default, not a silent fallthrough). |
| Html.AddChildNode() | 0x0141 | Not built. Returns 0. |
| Html.RemoveChildNode() | 0x0142 | Not built. Returns 0. |
| Html.SetAttribute() | 0x0143 | Not built. Returns 0. |
| Html.GetAttribute() | 0x0144 | Not built. Returns an empty span. |
| Html.ParseTree() | 0x0145 | Not built (needs an HTML parser). Returns 0. |
| Html.Encode() | 0x0146 | Pure: HTML entity-encodes a span (&amp;/&lt;/&gt;/&quot;/&#39;). Implemented on all 3 runtimes. |
| Html.Decode() | 0x0147 | Pure: the inverse of Encode. Implemented on all 3 runtimes. |
| Html.Serialize() | 0x0148 | Not built. Returns an empty span. |
| Html.QuerySelector() | 0x0149 | Not built. Returns 0. |

### Context.* (15 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Context.GetVerb() | 0x00E0 | Reserved/host-injected (live request/connection state, akin to Req.Method). Returns an empty span outside a real request context. |
| Context.GetPath() | 0x00E1 | Reserved/host-injected. Returns an empty span. |
| Context.GetHost() | 0x00E2 | Reserved/host-injected. Returns an empty span. |
| Context.GetPort() | 0x00E3 | Reserved/host-injected. Returns 0. |
| Context.GetRemoteAddr() | 0x00E4 | Reserved/host-injected. Returns an empty span. |
| Context.GetUser() | 0x00E5 | Reserved/host-injected. Returns an empty span. |
| Context.GetPermissions() | 0x00E6 | Reserved/host-injected. Returns an empty span. |
| Context.GetHeaders() | 0x00E7 | Reserved/host-injected. Returns an empty span. |
| Context.GetQueryString() | 0x00E8 | Reserved/host-injected. Returns an empty span. |
| Context.GetBody() | 0x00E9 | Reserved/host-injected. Returns an empty span. |
| Context.SetScratchValue() | 0x00EA | Reserved/host-injected. Returns 0. |
| Context.GetScratchValue() | 0x00EB | Reserved/host-injected. Returns 0. |
| Context.GetRequestId() | 0x00EC | Reserved/host-injected. Returns an empty span. |
| Context.GetClientCert() | 0x00ED | Reserved/host-injected. Returns an empty span. |
| Context.GetTraceId() | 0x00EE | Reserved/host-injected. Returns an empty span. |

---

## Strings & Numbers

### String.* (13 hooks)

| Method | Code | Description |
|--------|------|-------------|
| String.Concat() | 0x0080 | |
| String.Length() | 0x0081 | |
| String.Substring() | 0x0082 | |
| String.IndexOf() | 0x0083 | |
| String.Replace() | 0x0084 | |
| String.ToUpper() | 0x0085 | |
| String.ToLower() | 0x0086 | |
| String.Trim() | 0x0087 | |
| String.Split() | 0x0088 | Splits a span by a delimiter span into a fresh Map (int key 0..N-1 -> string part), reusing Map.* storage since the 2-in/1-out ABI has no array type. Returns the Map handle. Does not disturb the caller's active map. |
| String.Join() | 0x0089 | Joins the int-keyed 0..N-1 string parts of a Map (e.g. one returned by Split) with a separator span. Returns the joined span. |
| String.StartsWith() | 0x008A | |
| String.EndsWith() | 0x008B | |
| String.SetReplace() | 0x008C | |

### Number.* (11 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Number.Parse() | 0x0090 | |
| Number.ToString() | 0x0091 | |
| Number.ToHex() | 0x0092 | |
| Number.ToOctal() | 0x0093 | |
| Number.ToBinary() | 0x0094 | |
| Number.Abs() | 0x0095 | |
| Number.Floor() | 0x0096 | |
| Number.Ceiling() | 0x0097 | |
| Number.Round() | 0x0098 | |
| Number.Min() | 0x0099 | |
| Number.Max() | 0x009A | |

### Maths.* (12 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Maths.Sin() | 0x00A0 | |
| Maths.Cos() | 0x00A1 | |
| Maths.Tan() | 0x00A2 | |
| Maths.Sqrt() | 0x00A3 | |
| Maths.Power() | 0x00A4 | |
| Maths.Log() | 0x00A5 | |
| Maths.Log10() | 0x00A6 | |
| Maths.Exp() | 0x00A7 | |
| Maths.Random() | 0x00A8 | |
| Maths.RandomRange() | 0x00A9 | |
| Maths.Clamp() | 0x00AA | |
| Maths.Lerp() | 0x00AB | |

### DateTime.* (15 hooks)

| Method | Code | Description |
|--------|------|-------------|
| DateTime.Now() | 0x00B0 | |
| DateTime.UtcNow() | 0x00B1 | |
| DateTime.Parse() | 0x00B2 | |
| DateTime.Format() | 0x00B3 | |
| DateTime.AddSeconds() | 0x00B4 | |
| DateTime.AddMinutes() | 0x00B5 | |
| DateTime.AddHours() | 0x00B6 | |
| DateTime.AddDays() | 0x00B7 | |
| DateTime.GetDayOfWeek() | 0x00B8 | |
| DateTime.GetDayOfYear() | 0x00B9 | |
| DateTime.UnixTimestamp() | 0x00BA | |
| DateTime.DiffDays() | 0x00BB | |
| DateTime.Year() | 0x00BC | |
| DateTime.Month() | 0x00BD | |
| DateTime.Day() | 0x00BE | |

### Locale.* (7 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Locale.GetCurrentLocale() | 0x00C0 | |
| Locale.SetLocale() | 0x00C1 | |
| Locale.FormatCurrency() | 0x00C2 | |
| Locale.FormatNumber() | 0x00C3 | |
| Locale.FormatDate() | 0x00C4 | |
| Locale.FormatTime() | 0x00C5 | |
| Locale.Translate() | 0x00C6 | |

### Base64.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Base64.Encode() | 0x02D0 | |
| Base64.Decode() | 0x02D1 | |
| Base64.UrlDecode() | 0x02D2 | |

---

## Security

### Crypto.* (15 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Crypto.Sha256() | 0x00F0 | |
| Crypto.Sha512() | 0x00F1 | |
| Crypto.Blake2b() | 0x00F2 | |
| Crypto.Blake3() | 0x00F3 | |
| Crypto.HmacSha256() | 0x00F4 | |
| Crypto.HmacSha512() | 0x00F5 | |
| Crypto.Sign() | 0x00F6 | |
| Crypto.Verify() | 0x00F7 | |
| Crypto.Encrypt() | 0x00F8 | |
| Crypto.Decrypt() | 0x00F9 | |
| Crypto.GenerateKeyPair() | 0x00FA | |
| Crypto.DeriveKey() | 0x00FB | |
| Crypto.RandomBytes() | 0x00FC | |
| Crypto.Md5() | 0x00FD | |
| Crypto.Sha1() | 0x00FE | |

### X509.* (8 hooks)

| Method | Code | Description |
|--------|------|-------------|
| X509.FetchCertificate() | 0x0110 | Reserved/host-injected (needs a PKI trust store + entropy). Returns an empty span. |
| X509.StoreCertificate() | 0x0111 | Reserved/host-injected. Returns 0. |
| X509.GenerateCSR() | 0x0112 | Reserved/host-injected. Returns an empty span. |
| X509.GenerateKeyPair() | 0x0113 | Reserved/host-injected. Returns an empty span. |
| X509.VerifyCertChain() | 0x0114 | Reserved/host-injected. Returns 0. |
| X509.GetCertInfo() | 0x0115 | Reserved/host-injected. Returns an empty span. |
| X509.IsCertValid() | 0x0116 | Reserved/host-injected. Returns 0. |
| X509.GetKeyHandle() | 0x0117 | Reserved/host-injected. Returns 0. |

### Auth.* (10 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Auth.GetUserCredentials() | 0x0120 | Reserved/host-injected (needs an identity provider). Returns an empty span. |
| Auth.ValidateCredentials() | 0x0121 | Reserved/host-injected. Returns 0. |
| Auth.SwitchUserContext() | 0x0122 | Reserved/host-injected. Returns 0. |
| Auth.GetUserPermissions() | 0x0123 | Reserved/host-injected. Returns an empty span. |
| Auth.RequestToken() | 0x0124 | Reserved/host-injected. Returns an empty span. |
| Auth.GetToken() | 0x0125 | Reserved/host-injected. Returns an empty span. |
| Auth.ValidateToken() | 0x0126 | Reserved/host-injected. Returns 0. |
| Auth.SwitchTokenContext() | 0x0127 | Reserved/host-injected. Returns 0. |
| Auth.RefreshToken() | 0x0128 | Reserved/host-injected. Returns an empty span. |
| Auth.RevokeToken() | 0x0129 | Reserved/host-injected. Returns 0. |

### Principal.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Principal.Current() | 0x02A0 | |
| Principal.HasRole() | 0x02A1 | |
| Principal.Claims() | 0x02A2 | |

### Capability.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Capability.Has() | 0x02A3 | |
| Capability.Request() | 0x02A4 | |
| Capability.Drop() | 0x02A5 | |

### Sandbox.* (1 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Sandbox.Deny() | 0x02A6 | |

---

## Compression

### Compress.* (8 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Compress.BrotliCompress() | 0x0100 | |
| Compress.BrotliDecompress() | 0x0101 | |
| Compress.PicoCompress() | 0x0102 | |
| Compress.PicoDecompress() | 0x0103 | |
| Compress.GzipCompress() | 0x0104 | |
| Compress.GzipDecompress() | 0x0105 | |
| Compress.DeflateCompress() | 0x0106 | |
| Compress.DeflateDecompress() | 0x0107 | |

---

## Hardware & Devices

### Kernel.* (6 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Kernel.WaitIRQ() | 0x0001 | Real: reuses the same cooperative-yield halt as the raw OP_WAIT opcode, on all 3 runtimes. |
| Kernel.WaitSWIRQ() | 0x0002 | Real: same mechanism as WaitIRQ. |
| Kernel.FireSWIRQ() | 0x0003 | Real: signals a software IRQ (ack-only on the embedded C runtime, which carries no debug-log list; Python/JS also append a log line). |
| Kernel.ProfileStart() | 0x0004 | Real on all 3 runtimes: reuses the Log.* table (deterministic, sequence-ordered). |
| Kernel.ProfileEnd() | 0x0005 | Same status as ProfileStart. |
| Kernel.TracePoint() | 0x0006 | Same status as ProfileStart. |

### Queue.* (5 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Queue.Dequeue() | 0x0010 | |
| Queue.Enqueue() | 0x0011 | |
| Queue.Depth() | 0x0012 | |
| Queue.DequeueBatch() | 0x0013 | |
| Queue.EnqueueBatch() | 0x0014 | |

### Random.* (1 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Random.U32() | 0x0020 | |

### Gpio.* (7 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Gpio.Count() | 0x0150 | |
| Gpio.SetDir() | 0x0151 | |
| Gpio.GetDir() | 0x0152 | |
| Gpio.SetPull() | 0x0153 | |
| Gpio.GetPull() | 0x0154 | |
| Gpio.Write() | 0x0155 | |
| Gpio.Read() | 0x0156 | |

### Device.* (4 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Device.Open() | 0x0168 | |
| Device.Caps() | 0x0169 | |
| Device.Close() | 0x016A | |
| Device.Status() | 0x016B | |

### Stream.* (8 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Stream.Open() | 0x0170 | |
| Stream.Next() | 0x0171 | |
| Stream.Span() | 0x0172 | |
| Stream.Submit() | 0x0173 | |
| Stream.Release() | 0x0174 | |
| Stream.Close() | 0x0175 | |
| Stream.SetSlice() | 0x0176 | |
| Stream.Slice() | 0x0177 | |

---

## Capsules & IPC

### Pack.* (1 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Pack.Use() | 0x0160 | Real: a lightweight "active pack" selector (independent of Storage's own pack context), on all 3 runtimes. |

### Card.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Card.Read() | 0x0161 | Reserved/host-injected (physical card reader hardware). Returns 0. |
| Card.Write() | 0x0162 | Reserved/host-injected. Returns 0. |
| Card.Address() | 0x0163 | Reserved/host-injected. Returns 0. |

### Fifo.* (4 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Fifo.Open() | 0x0164 | Real: opens an independent named byte-channel FIFO, returns a fresh channel handle. No host state -- deterministic on all 3 runtimes. Distinct from Queue.* (fixed 8-channel int FIFO). |
| Fifo.Send() | 0x0165 | Real: pushes a span onto the channel. |
| Fifo.Recv() | 0x0166 | Real: pops the oldest span off the channel (empty span if empty/unknown). |
| Fifo.Poll() | 0x0167 | Real: returns the number of buffered messages on the channel. |

### Capsule.* (5 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Capsule.Call() | 0x02C0 | |
| Capsule.Schedule() | 0x02C1 | |
| Capsule.Jump() | 0x02C2 | |
| Capsule.LoadModule() | 0x02C3 | |
| Capsule.RunModule() | 0x02C4 | |

---

## Events & UI

### Event.* (10 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Event.Post() | 0x0180 | |
| Event.Next() | 0x0181 | |
| Event.Type() | 0x0182 | |
| Event.Target() | 0x0183 | |
| Event.Data() | 0x0184 | |
| Event.SetData() | 0x0185 | |
| Event.Count() | 0x0186 | |
| Event.SetSlice() | 0x01B3 | |
| Event.DataSlice() | 0x01B4 | |
| Event.DataLen() | 0x01B5 | |

### Ui.* (12 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Ui.Window() | 0x0188 | |
| Ui.Panel() | 0x0189 | |
| Ui.Label() | 0x018A | |
| Ui.Button() | 0x018B | |
| Ui.TextBox() | 0x018C | |
| Ui.Checkbox() | 0x018D | |
| Ui.Pos() | 0x018E | |
| Ui.Size() | 0x018F | |
| Ui.SetText() | 0x0190 | |
| Ui.SetId() | 0x0191 | |
| Ui.SetValue() | 0x0192 | |
| Ui.Serialize() | 0x0193 | |

### Assert.* (5 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Assert.Eq() | 0x0178 | |
| Assert.True() | 0x0179 | |
| Assert.Count() | 0x017A | |
| Assert.Failed() | 0x017B | |
| Assert.Reset() | 0x017C | |

---

## AI & Inference

### Tensor.* (22 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Tensor.SetShape() | 0x01E0 | |
| Tensor.DotI8() | 0x01E1 | |
| Tensor.MatVecI8() | 0x01E2 | |
| Tensor.AddI32() | 0x01E3 | |
| Tensor.MulI32() | 0x01E4 | |
| Tensor.ScaleI32() | 0x01E5 | |
| Tensor.ReluI32() | 0x01E6 | |
| Tensor.RmsNormI32() | 0x01E7 | |
| Tensor.RoPEI32() | 0x01E8 | |
| Tensor.SoftmaxI32() | 0x01E9 | |
| Tensor.ArgMaxI32() | 0x01EA | |
| Tensor.HasAccel() | 0x01EB | |
| Tensor.Map() | 0x0370 | Map host storage/device memory as an opaque tensor. |
| Tensor.View() | 0x0371 | Create a host-defined tensor view. |
| Tensor.Gemm() | 0x0372 | Host-accelerated matrix multiplication. |
| Tensor.Reduce() | 0x0373 | Host-defined tensor reduction. |
| Tensor.Elementwise() | 0x0374 | Host-defined elementwise transform. |
| Tensor.Add() | 0x0382 | Elementwise F32 tensor addition. |
| Tensor.Mul() | 0x0383 | Elementwise F32 tensor multiplication. |
| Tensor.RmsNorm() | 0x0384 | F32 RMSNorm using a gamma tensor. |
| Tensor.SwiGLU() | 0x0385 | Fused `SiLU(gate) * up` tensor operation. |
| Tensor.Release() | 0x0386 | Release an opaque tensor/shard/CAT-Q provider handle. |

### CatQ.* (5 hooks)

| Method | Code | Description |
|--------|------|-------------|
| CatQ.Calibrate() | 0x0375 | Build a CAT-Q calibration context. |
| CatQ.Optimize() | 0x0376 | Optimize a weight tensor within a CAT-Q context. |
| CatQ.Ternarize() | 0x0377 | Produce ternary weights. |
| CatQ.Pack() | 0x0378 | Pack ternary weights and scaling metadata. |
| CatQ.CalibrateTarget() | 0x038A | Attach a target tensor to a calibration context. |

### Block.* (13 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Block.Ready() | 0x03D0 | Test whether the block device is mounted and usable. |
| Block.BlockSize() | 0x03D1 | Return the device block size in bytes. |
| Block.SizeLow() | 0x03D2 | Return the low 32 bits of device size. |
| Block.SizeHigh() | 0x03D3 | Return the high 32 bits of device size. |
| Block.SetOffset() | 0x03D4 | Set the byte offset using low/high 32-bit registers. |
| Block.Read() | 0x03D5 | Read a bounded byte span from the current offset. |
| Block.Write() | 0x03D6 | Write a bounded byte span at the current offset. |
| Block.Sync() | 0x03D7 | Flush pending device writes. |
| Block.Resize() | 0x03D8 | Resize the device using low/high 32-bit registers. |
| Block.SetLba() | 0x03D9 | Set the current logical block address. |
| Block.ReadBlocks() | 0x03DA | Read a bounded number of whole blocks. |
| Block.WriteBlocks() | 0x03DB | Write a bounded number of whole blocks. |
| Block.Status() | 0x03DC | Return the last block-device status code. |

### Async.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Async.Submit() | 0x0379 | Submit a coarse host job. |
| Async.Wait() | 0x037A | Wait for completion or timeout. |
| Async.Result() | 0x037B | Retrieve a completed result handle/span. |

### Shard.* (2 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Shard.Load() | 0x037C | Load or map a model shard. |
| Shard.Save() | 0x037D | Persist a transformed model shard. |

### Media.* (11 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Media.SetShape() | 0x0360 | Configure grayscale frame dimensions. |
| Media.GrayDeltaEncode() | 0x0361 | Encode reversible grayscale deltas. |
| Media.GrayDeltaDecode() | 0x0362 | Decode reversible grayscale deltas. |
| Media.H264Residual() | 0x0363 | Produce a modulo-256 prediction residual. |
| Media.H264Restore() | 0x0364 | Restore grayscale bytes from a residual. |
| Media.HasAccel() | 0x0365 | Report whether a native media accelerator is installed. |
| Media.HevcConfigure() | 0x0366 | Configure a host HEVC decoder. |
| Media.HevcDecode() | 0x0367 | Decode through a host HEVC provider. |
| Media.HasHevc() | 0x0368 | Report whether a host HEVC decoder is available. |
| Media.GrayXorResidual() | 0x0369 | Produce an XOR prediction residual. |
| Media.GrayXorRestore() | 0x036A | Restore grayscale bytes from an XOR residual. |

### MoE.* (3 hooks)

| Method | Code | Description |
|--------|------|-------------|
| MoE.Forward() | 0x0387 | Route one token and stream only its selected experts. |
| MoE.SelectedCount() | 0x0388 | Return the number of selected experts. |
| MoE.SelectedExpert() | 0x0389 | Inspect one selected expert ID. |

### BitLinear.* (10 hooks)

| Method | Code | Description |
|--------|------|-------------|
| BitLinear.SetShape() | 0x01F0 | |
| BitLinear.MatVecTernary() | 0x01F1 | |
| BitLinear.MatVecBitmap() | 0x01F2 | |
| BitLinear.MatVecBase3() | 0x01F3 | |
| BitLinear.HasFormat() | 0x01F4 | |
| BitLinear.MatVecTernaryBlock() | 0x0274 | Run ternary matvec against the active model block. |
| BitLinear.MatVecBitmapBlock() | 0x0275 | Run bitmap matvec against the active model block. |
| BitLinear.MatVecBase3Block() | 0x0276 | Run base-3 matvec against the active model block. |
| BitLinear.MatMulBitmapBatch() | 0x036B | Run bitmap weights over a batch of int8 vectors. |
| BitLinear.MatVecCatQ() | 0x0381 | Scaled CAT-Q packed-weight matrix-vector projection. |

### Quant.* (5 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Quant.AbsMax() | 0x0228 | |
| Quant.QuantI8() | 0x0229 | |
| Quant.DequantI8() | 0x022A | |
| Quant.ApplyScale() | 0x022B | |
| Quant.GroupScale() | 0x022C | |

### Attention.* (4 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Attention.SetShape() | 0x0250 | |
| Attention.Scores() | 0x0251 | |
| Attention.Mix() | 0x0252 | |
| Attention.Attend() | 0x0253 | |

### Tokenizer.* (7 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Tokenizer.SetVocab() | 0x0210 | |
| Tokenizer.EncodeBytes() | 0x0211 | |
| Tokenizer.EncodeTrie() | 0x0212 | |
| Tokenizer.DecodeBytes() | 0x0213 | |
| Tokenizer.DecodeTrie() | 0x0214 | |
| Tokenizer.Count() | 0x0215 | |
| Tokenizer.Token() | 0x0216 | |

### Model.* (9 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Model.SetConfig() | 0x0220 | |
| Model.GetConfig() | 0x0221 | |
| Model.TensorView() | 0x0222 | |
| Model.TensorOffset() | 0x0223 | |
| Model.TensorRows() | 0x0224 | |
| Model.TensorCols() | 0x0225 | |
| Model.TensorFormat() | 0x0226 | |
| Model.ReadTensor() | 0x0227 | |
| Model.ReadTensorRow() | 0x0270 | |

### Kv.* (12 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Kv.SetShape() | 0x0230 | |
| Kv.WriteK() | 0x0231 | |
| Kv.WriteV() | 0x0232 | |
| Kv.ReadK() | 0x0233 | |
| Kv.ReadV() | 0x0234 | |
| Kv.Len() | 0x0235 | |
| Kv.Clear() | 0x0236 | |
| Kv.SetHead() | 0x0237 | |
| Kv.WriteKH() | 0x0238 | |
| Kv.WriteVH() | 0x0239 | |
| Kv.ReadKH() | 0x023A | |
| Kv.ReadVH() | 0x023B | |

### Sampling.* (4 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Sampling.ArgMax() | 0x0240 | |
| Sampling.TopK() | 0x0241 | |
| Sampling.Temperature() | 0x0242 | |
| Sampling.ArgMaxRows() | 0x0243 | |

---

## OS Worker

### Process.* (8 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Process.Self() | 0x0280 | |
| Process.Parent() | 0x0281 | |
| Process.Spawn() | 0x0282 | |
| Process.Exit() | 0x0283 | |
| Process.Kill() | 0x0284 | |
| Process.Status() | 0x0285 | |
| Process.Wait() | 0x0286 | |
| Process.Args() | 0x0287 | |

### Env.* (4 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Env.Get() | 0x0288 | |
| Env.Set() | 0x0289 | |
| Env.Count() | 0x028A | |
| Env.Key() | 0x028B | |

### Timer.* (4 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Timer.After() | 0x0290 | |
| Timer.Every() | 0x0291 | |
| Timer.Cancel() | 0x0292 | |
| Timer.Elapsed() | 0x0293 | |

### Scheduler.* (1 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Scheduler.Tick() | 0x0294 | |

### Error.* (8 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Error.SetHandler() | 0x02B0 | Pushes a handler PC onto the exception-handler stack (real try/except mechanism -- see docs/EXCEPTION_ENGINE.md). Real on all 3 bytecode VMs (Python/JS/C). |
| Error.HasHandler() | 0x02B1 | Returns 1 if a handler is active (checks the top of the handler stack's truthiness). All 3 bytecode VMs. |
| Error.Code() | 0x02B2 | Returns the last fault/raise code. All 3 bytecode VMs. |
| Error.Detail() | 0x02B3 | Returns the last fault/raise detail. All 3 bytecode VMs. |
| Error.Resume() | 0x02B4 | Returns the PC to resume at after a caught fault. All 3 bytecode VMs. |
| Error.Clear() | 0x02B5 | Clears the last fault/raise state. All 3 bytecode VMs. |
| Error.Raise() | 0x02B6 | Raises to the nearest handler (or propagates as an uncaught fault). All 3 bytecode VMs. |
| Error.PopHandler() | 0x02B7 | Pops the handler stack (restores the enclosing try's handler, if any). All 3 bytecode VMs. |

**Now implemented on the C VM (`vm/picovm.c`) interpreter too** (fixed in the
namespace-equalization pass's follow-up -- see docs/EXCEPTION_ENGINE.md).
`laddr` (the label-address IL construct) needed no new C opcode -- it's a
purely compile-time bytecode-assembly trick (plain `SUB`/`ADD`/`MUL` words);
the real gaps were the `Error.*` dispatch itself and a `pending_jump` channel
for hooks/caught-faults to redirect the interpreter's `pc`. **Not
implemented in native-C transpile** (`lower_to_c` → straight-line `goto`-
based C, no bytecode PC to jump to at all) or native-JS transpile — both
still raise a clear `ValueError` at compile time (see
docs/EXCEPTION_ENGINE.md's scope section) — that limitation is unrelated to
and unaffected by the interpreter fix.

---

## System Info

### Environment.* (9 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Environment.GetOsVersion() | 0x00D0 | Reserved/host-injected (OS/host facts). Returns an empty span. |
| Environment.GetCpuCount() | 0x00D1 | Reserved/host-injected. Returns 0. |
| Environment.GetMemoryTotal() | 0x00D2 | Reserved/host-injected. Returns 0. |
| Environment.GetMemoryFree() | 0x00D3 | Reserved/host-injected. Returns 0. |
| Environment.GetHostname() | 0x00D4 | Reserved/host-injected. Returns an empty span. |
| Environment.GetTimeZone() | 0x00D5 | Reserved/host-injected. Returns an empty span. |
| Environment.GetProcessId() | 0x00D6 | Reserved/host-injected. Returns 0. |
| Environment.GetThreadId() | 0x00D7 | Reserved/host-injected. Returns 0. |
| Environment.GetElapsedTime() | 0x00D8 | Reserved/host-injected. Returns 0. |

### Status.* (1 hooks)

| Method | Code | Description |
|--------|------|-------------|
| Status.Last() | 0x005E | |

---

## Structured Data — Map, Parsing & Binary Serialization

First-class dictionary (`Map`) plus string/bytes → structured `Map` parsers.
Full design + semantics in [docs/MAP.md](MAP.md). Implemented identically on the
Python, JS and C VMs (bit-identical output).

### Map.* (27 hooks) — active-handle dictionary

Keys: int / string / hash (FNV-1a). Values: int / string / null. Insertion-order
enumeration. `New`/`Use` select the active map; every other op acts on it (so all
ops fit the 2-arg host-call ABI — no compiler changes in any dialect).

| Method | Code | Description |
|--------|------|-------------|
| Map.New() | 0x0320 | create empty map, set active -> handle |
| Map.Use(h) | 0x033A | select the active map |
| Map.Free(h) | 0x0321 | release a map |
| Map.Clear() | 0x0322 | empty the active map |
| Map.Count() | 0x0323 | entry count |
| Map.Hash(span) | 0x0324 | FNV-1a 32-bit |
| Map.PutII(k,v) / GetII(k) | 0x0325 / 0x0326 | int->int |
| Map.HasI(k) / DelI(k) | 0x0327 / 0x0328 | int-key has / delete |
| Map.PutIS(k,vSpan) / GetIS(k) | 0x0329 / 0x032A | int->string |
| Map.PutNullI(k) / IsNullI(k) | 0x032B / 0x032C | int->null |
| Map.PutSI(kSpan,v) / GetSI(kSpan) | 0x032D / 0x032E | string->int |
| Map.HasS(kSpan) / DelS(kSpan) | 0x032F / 0x0330 | string-key has / delete |
| Map.PutSS(kSpan,vSpan) / GetSS(kSpan) | 0x0331 / 0x0332 | string->string |
| Map.PutNullS(kSpan) / IsNullS(kSpan) | 0x0333 / 0x0334 | string->null |
| Map.KeyAt(i) / KeySpanAt(i) | 0x0335 / 0x0336 | enumerate keys |
| Map.ValAt(i) / ValSpanAt(i) | 0x0337 / 0x0338 | enumerate values |
| Map.ValIsSpan(i) | 0x0339 | value at index is a string |

### Json.* / Binary.* (parsing & serialization)

| Method | Code | Description |
|--------|------|-------------|
| Json.Parse(span) | 0x0340 | flat JSON object -> Map |
| Binary.ParseCard(span) | 0x0341 | PicoBinarySerializer PSC1 card -> Map |
| Binary.SerializeCard() | 0x0342 | active Map -> PSC1 card |
| Binary.ParseEntity(blob,schema) | 0x0343 | BSO1 (BareMetal.Binary) entity -> Map |
| Binary.SerializeEntity(data,schema) | 0x0344 | Map -> BSO1 entity (signed if key set) |
| Binary.SetKey(span) | 0x0345 | BSO1 HMAC-SHA256 signing key |
| Binary.Verify(blob) | 0x0346 | verify BSO1 HMAC signature -> 0\|1 |

### Http transport (4 hooks) — used by the workflow WEB action

| Method | Code | Description |
|--------|------|-------------|
| Http.Request(reqMap,body) | 0x0138 | send request (headers as a Map) -> response handle |
| Http.RespStatus(resp) | 0x0139 | response status code |
| Http.RespHeaders(resp) | 0x013A | response headers -> enumerable Map |
| Http.RespBody(resp,outDesc) | 0x013B | response body span |
