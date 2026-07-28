# AGT Qt5 Design System

## Theme Tokens

Each `resources/themes/<theme-id>/theme.json` supplies:

```text
background surface text mutedText border accent success warning danger
```

`theme.qss` consumes tokens through `UiThemeManager`. Add a token before
duplicating a color across widgets. Keep light and dark manifests structurally
identical. Use no online font, icon, or CDN resource.

## Spacing And Shape

- Base comfortable spacing: 8 px; compact spacing: 4 px.
- Section spacing: 12-16 px.
- Command controls: at least 36 px high.
- Card/control radius: at most 8 px; current default is 4 px.
- Use stable widths or grids for button groups so translated text does not
  resize the shell.
- Keep letter spacing at 0.

## Typography

Use the existing system-font fallback and `UiLanguage::Text(zh_CN, en_US)`.
Keep panel headings compact. Do not use hero-scale typography in an operator
console. Every new operator label, warning, confirmation, and result must have
Chinese and English text through the same language boundary.

## Status Semantics

Never communicate state by color alone. Pair status color with explicit text:

| Meaning | Token | Text examples |
| --- | --- | --- |
| Ready/success | `success` | READY, 已连接, 已完成 |
| Attention | `warning` | DEGRADED, 等待, 受限 |
| Failure/danger | `danger` | FAILED, 急停, 已断开 |
| Unknown | `mutedText` | UNKNOWN, 未知, 无新鲜证据 |

Unknown or stale backend evidence remains UNKNOWN. Node presence is not
readiness.

## Dangerous Actions

Require confirmation for motion enable, Mission cancel, mapping discard, map
archive/delete/purge, experiment interruption, and emergency-stop reset.
Confirmation text names the operation and lets the backend enforce final
dependency/safety checks.

## Shell And Viewports

`control-center-v1` uses a status bar, 164 px primary navigation, expanding
legacy map/dock workspace, and 270 px context rail. `legacy` remains a fallback
using the same channel and ViewModels.

At 1366x768, keep the map/workspace usable and split long command groups into
rows. At 1920x1080, let the center workspace expand rather than enlarging fixed
sidebars. Use scrollable tables for variable asset lists; never overlap status,
navigation, workspace, or context controls.

Theme and density may restyle the shell. They never hide a safety state or
change a capability.
