"""PyQt dialog for selecting Workbench map-resource bundle contents."""

from __future__ import annotations

from pathlib import Path

from PyQt5.QtWidgets import (
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from .resource_bundle import ResourceBundleGroup, ResourceBundleSelection


class ResourceBundleDialog(QDialog):
    """One-shot selector for an immutable Workbench resource bundle."""

    def __init__(
        self,
        groups: list[ResourceBundleGroup],
        *,
        original_pcd_available: bool,
        default_parent: Path | str,
        default_name: str,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("另存为地图资源包")
        self.resize(620, 720)
        self._groups = list(groups)
        self._checks: dict[str, QCheckBox] = {}

        root = QVBoxLayout(self)
        intro = QLabel(
            "选择本次需要持久化的地图资源。正式 Navigation revision 会保持 "
            "Generated + Accepted + Derivation 完整结构；未勾选的派生层不会写入资源包。"
        )
        intro.setWordWrap(True)
        root.addWidget(intro)

        destination_box = QGroupBox("资源包位置")
        destination_layout = QFormLayout(destination_box)
        parent_row = QWidget()
        parent_layout = QHBoxLayout(parent_row)
        parent_layout.setContentsMargins(0, 0, 0, 0)
        self._parent_edit = QLineEdit(str(Path(default_parent).expanduser()))
        browse = QPushButton("选择…")
        browse.clicked.connect(self._browse_parent)
        parent_layout.addWidget(self._parent_edit, 1)
        parent_layout.addWidget(browse)
        destination_layout.addRow("目标目录：", parent_row)
        self._name_edit = QLineEdit(str(default_name))
        destination_layout.addRow("资源包名称：", self._name_edit)
        root.addWidget(destination_box)

        resources_box = QGroupBox("需要保存的资源")
        resources_layout = QVBoxLayout(resources_box)
        for group in self._groups:
            check = QCheckBox(group.label)
            check.setEnabled(bool(group.available))
            check.setChecked(bool(group.available and group.default_selected))
            if not group.available:
                reason = group.unavailable_reason or "当前 Workbench 尚未生成该资源"
                check.setText(f"{group.label} 〔未生成〕")
                check.setToolTip(reason)
            resources_layout.addWidget(check)
            self._checks[group.key] = check

        resources_layout.addSpacing(8)
        self._copy_original = QCheckBox("复制原始输入 PCD 到资源包（默认关闭）")
        self._copy_original.setEnabled(bool(original_pcd_available))
        self._copy_original.setChecked(False)
        if not original_pcd_available:
            self._copy_original.setText("复制原始输入 PCD 到资源包 〔本次会话未知〕")
        resources_layout.addWidget(self._copy_original)
        raw_note = QLabel(
            "关闭时不会复制原始 PCD，只在 manifest 中记录原始路径与 SHA256；"
            "开启后才额外写入 pointcloud/original.pcd。"
        )
        raw_note.setWordWrap(True)
        resources_layout.addWidget(raw_note)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(resources_box)
        root.addWidget(scroll, 1)

        preset_row = QHBoxLayout()
        recommended = QPushButton("推荐选择")
        select_all = QPushButton("全选")
        clear_all = QPushButton("全部取消")
        recommended.clicked.connect(self._recommended)
        select_all.clicked.connect(self._select_all)
        clear_all.clicked.connect(self._clear_all)
        preset_row.addWidget(recommended)
        preset_row.addWidget(select_all)
        preset_row.addWidget(clear_all)
        preset_row.addStretch(1)
        root.addLayout(preset_row)

        self._validation = QLabel("")
        self._validation.setWordWrap(True)
        root.addWidget(self._validation)

        buttons = QDialogButtonBox(QDialogButtonBox.Cancel | QDialogButtonBox.Save)
        buttons.button(QDialogButtonBox.Save).setText("另存资源包")
        buttons.accepted.connect(self._accept_if_valid)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

    def _browse_parent(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图资源包目标目录",
            self._parent_edit.text().strip() or str(Path.cwd()),
        )
        if selected:
            self._parent_edit.setText(selected)

    def _recommended(self) -> None:
        for group in self._groups:
            self._checks[group.key].setChecked(
                bool(group.available and group.default_selected)
            )
        self._copy_original.setChecked(False)

    def _select_all(self) -> None:
        for group in self._groups:
            self._checks[group.key].setChecked(bool(group.available))
        if self._copy_original.isEnabled():
            self._copy_original.setChecked(True)

    def _clear_all(self) -> None:
        for check in self._checks.values():
            check.setChecked(False)
        self._copy_original.setChecked(False)

    def _accept_if_valid(self) -> None:
        parent = self._parent_edit.text().strip()
        name = self._name_edit.text().strip()
        if not parent:
            self._validation.setText("请选择目标目录")
            return
        if not name or Path(name).name != name or name in {".", ".."}:
            self._validation.setText("资源包名称只能是一个非空的单层目录名")
            return
        self._validation.clear()
        self.accept()

    def parent_dir(self) -> Path:
        return Path(self._parent_edit.text().strip()).expanduser()

    def bundle_name(self) -> str:
        return self._name_edit.text().strip()

    def selection(self) -> ResourceBundleSelection:
        selected = frozenset(
            key for key, check in self._checks.items() if check.isChecked()
        )
        return ResourceBundleSelection(
            selected_groups=selected,
            copy_original_pcd=bool(self._copy_original.isChecked()),
        )
