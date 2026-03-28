# Send and receive commands via the USB interface

class GSS6100:
    VID = 0x0525
    PID = 0xA105

    EP_OUT = 0x01
    EP_IN_RESP = 0x82
    EP_IN_EVT = 0x83

    def __init__(
        self,
        default_terminator="\r",
        first_timeout_ms=700,
        inter_packet_timeout_ms=80,
        overall_timeout_ms=2000,
    ):
        self.dev = usb.core.find(idVendor=self.VID, idProduct=self.PID)
        if self.dev is None:
            raise RuntimeError("GSS6100 not found")

        self.dev.set_configuration()

        self.default_terminator = default_terminator
        self.first_timeout_ms = int(first_timeout_ms)
        self.inter_packet_timeout_ms = int(inter_packet_timeout_ms)
        self.overall_timeout_ms = int(overall_timeout_ms)

    def _read_once(self, ep, size=64, timeout_ms=50):
        try:
            return bytes(self.dev.read(ep, size, timeout=timeout_ms))
        except usb.core.USBTimeoutError:
            return None

    def _drain(self, ep, timeout_ms=10, max_reads=100):
        out = []
        for _ in range(max_reads):
            data = self._read_once(ep, timeout_ms=timeout_ms)
            if data is None:
                break
            out.append(data)
        return out

    def clear_response_queue(self):
        self._drain(self.EP_IN_RESP, timeout_ms=10, max_reads=50)

    def send(self, cmd: str, terminator=None):
        if terminator is None:
            terminator = self.default_terminator
        payload = cmd.encode("ascii") + terminator.encode("ascii")
        self.dev.write(self.EP_OUT, payload, timeout=1000)

    def query_raw(self, cmd: str, terminator=None):
        if terminator is None:
            terminator = self.default_terminator

        self.clear_response_queue()
        self.send(cmd, terminator=terminator)

        response = bytearray()

        first = self._read_once(self.EP_IN_RESP, timeout_ms=self.first_timeout_ms)
        if first is None:
            raise TimeoutError(f"No response to {cmd!r}")

        response.extend(first)

        if b"\x00" in response:
            return bytes(response)

        deadline = time.monotonic() + self.overall_timeout_ms / 1000.0
        while time.monotonic() < deadline:
            data = self._read_once(self.EP_IN_RESP, timeout_ms=self.inter_packet_timeout_ms)
            if data is None:
                break
            response.extend(data)
            if b"\x00" in response:
                break

        return bytes(response)

    @staticmethod
    def decode(raw: bytes) -> str:
        raw = raw.split(b"\x00", 1)[0]
        return raw.decode("ascii", errors="replace").rstrip("\r\n")

    def query(self, cmd: str, terminators=("\r", "\n", "\r\n"), retries=2, settle_ms=40):
        last_exc = None

        for term in terminators:
            for _ in range(retries):
                try:
                    raw = self.query_raw(cmd, terminator=term)
                    txt = self.decode(raw)
                    time.sleep(settle_ms / 1000.0)
                    return txt
                except TimeoutError as e:
                    last_exc = e
                    time.sleep(settle_ms / 1000.0)

        raise last_exc if last_exc else TimeoutError(f"No response to {cmd!r}")

    def write(self, cmd: str, terminator="\r", settle_ms=80):
        # Best-effort write: clear stale responses, send, allow device to settle.
        self.clear_response_queue()
        self.send(cmd, terminator=terminator)
        time.sleep(settle_ms / 1000.0)

    def identify(self):
        return self.query("*IDN?")

    def get_level(self) -> str:
        return self.query("LEVL ?")

    def set_level(self, level_dbm: float) -> str:
        # Format matches manual examples like LEVL -15.0
        cmd = f"LEVL {level_dbm:.1f}"
        self.write(cmd)
        # Read back from device
        return self.get_level()

# usage>
# g = GSS6100()
# print("ID:    ", g.identify())
# print("ID:    ", g.query("*IDN?"))
