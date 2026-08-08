"""Read a Nav2 trinary map into the existing offline footprint validator model."""

from pathlib import Path

from agt_coverage_planning.path_validator import GridMap

from .contracts import AssetContractError, load_yaml_mapping


def load_nav2_grid(map_yaml_path: str | Path) -> GridMap:
    yaml_path = Path(map_yaml_path).expanduser().resolve()
    config = load_yaml_mapping(yaml_path)
    if str(config.get("mode", "trinary")).lower() != "trinary":
        raise AssetContractError(
            "map_mode_unsupported", "offline feasibility currently requires Nav2 trinary mode"
        )
    resolution = float(config.get("resolution", 0.0))
    origin = config.get("origin")
    if resolution <= 0.0 or not isinstance(origin, list) or len(origin) != 3:
        raise AssetContractError("map_geometry_invalid", "map resolution/origin are invalid")
    image_path = (yaml_path.parent / str(config.get("image", ""))).resolve()
    width, height, pixels, max_value = _read_pgm(image_path)
    negate = bool(int(config.get("negate", 0)))
    occupied_thresh = float(config.get("occupied_thresh", 0.65))
    free_thresh = float(config.get("free_thresh", 0.196))
    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise AssetContractError("map_threshold_invalid", "free/occupied thresholds are invalid")

    rows = []
    for source_row in range(height - 1, -1, -1):
        offset = source_row * width
        for column in range(width):
            pixel = pixels[offset + column]
            normalized = float(pixel) / float(max_value)
            occupancy_probability = normalized if negate else 1.0 - normalized
            if occupancy_probability > occupied_thresh:
                rows.append(100)
            elif occupancy_probability < free_thresh:
                rows.append(0)
            else:
                rows.append(-1)
    return GridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        data=tuple(rows),
        frame_id="map",
    )


def _read_pgm(path: Path) -> tuple[int, int, list[int], int]:
    if not path.is_file():
        raise AssetContractError("pgm_missing", f"PGM is missing: {path}")
    data = path.read_bytes()
    magic, width, height, max_value, offset = _pgm_header(data)
    width = int(width)
    height = int(height)
    max_value = int(max_value)
    if width <= 0 or height <= 0 or not 0 < max_value <= 65535:
        raise AssetContractError("pgm_header_invalid", "PGM dimensions/max value are invalid")
    count = width * height
    if magic == b"P2":
        body = data[offset:].decode("ascii")
        tokens = []
        for line in body.splitlines():
            line = line.split("#", 1)[0]
            tokens.extend(line.split())
        if len(tokens) != count:
            raise AssetContractError("pgm_size_mismatch", "P2 pixel count does not match dimensions")
        pixels = [int(token) for token in tokens]
    else:
        bytes_per_value = 1 if max_value < 256 else 2
        body = data[offset:]
        if len(body) != count * bytes_per_value:
            raise AssetContractError("pgm_size_mismatch", "P5 byte count does not match dimensions")
        if bytes_per_value == 1:
            pixels = list(body)
        else:
            pixels = [int.from_bytes(body[index:index + 2], "big") for index in range(0, len(body), 2)]
    if any(value < 0 or value > max_value for value in pixels):
        raise AssetContractError("pgm_value_invalid", "PGM contains values outside max value")
    return width, height, pixels, max_value


def _pgm_header(data: bytes) -> tuple[bytes, bytes, bytes, bytes, int]:
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index >= len(data):
            raise AssetContractError("pgm_header_invalid", "PGM header ended early")
        if data[index] == ord("#"):
            newline = data.find(b"\n", index)
            if newline < 0:
                raise AssetContractError("pgm_header_invalid", "unterminated PGM comment")
            index = newline + 1
            continue
        start = index
        while index < len(data) and data[index] not in b" \t\r\n#":
            index += 1
        tokens.append(data[start:index])
    if tokens[0] not in {b"P2", b"P5"}:
        raise AssetContractError("pgm_magic_invalid", "only P2/P5 PGM is supported")
    if tokens[0] == b"P5":
        if index >= len(data) or data[index] not in b" \t\r\n":
            raise AssetContractError("pgm_header_invalid", "P5 header lacks raster separator")
        if data[index:index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
    else:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
    return tokens[0], tokens[1], tokens[2], tokens[3], index
