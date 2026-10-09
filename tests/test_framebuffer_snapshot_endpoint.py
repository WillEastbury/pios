"""Pin the bounded, authenticated raw framebuffer snapshot contract."""
from pathlib import Path

kernel = (Path(__file__).resolve().parent.parent / "src" / "kernel.c").read_text()
route = kernel.index('http_request_path_is(req, req_len, "/api/framebuffer.raw")')
auth = kernel.rindex("http_admin_authorized", 0, route)
status = kernel.index("if (route == HTTP_ROUTE_STATUS)", route)
body = kernel[route:status]

assert auth < route
assert "fb_display_info(&base, &width, &height, &pitch, &size)" in body
assert "pitch < width * 4U" in body
assert "bytes > size" in body and "bytes > PIOS_FB_BACK_SIZE" in body
assert "http_static_body = (const u8 *)(usize)base" in body
assert "X-PIOS-Pixel-Format: BGRX8888" in body
assert "X-PIOS-Width:" in body and "X-PIOS-Height:" in body
assert "X-PIOS-Pitch:" in body and "Content-Length:" in body
print("Framebuffer snapshot: authenticated, bounded BGRX stream contract passed")
