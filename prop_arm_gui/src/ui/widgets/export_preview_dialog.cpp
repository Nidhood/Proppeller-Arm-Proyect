#include "prop_arm_gui/ui/widgets/export_preview_dialog.hpp"

#include "prop_arm_gui/core/logging/data_exporter.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <QVBoxLayout>

#include <algorithm>

namespace prop_arm::ui::widgets {

namespace {

QStandardItem* mkItem(const QString& txt) {
    auto* it = new QStandardItem(txt);
    it->setEditable(false);
    return it;
}

QString domainToUi(prop_arm::models::Domain d) {
    return (d == prop_arm::models::Domain::Real) ? "Real" : "Sim";
}

}  // namespace

ExportPreviewDialog::ExportPreviewDialog(QWidget* parent)
    : QDialog(parent) {

    // Keys must match what you pass into CsvExportOptions.columns
    columns_ = {
        {"Domain", "Domain (real/sim)"},
        {"t_rel_s", "t_rel_s (relative seconds)"},
        {"arm_angle_rad", "arm_angle_rad"},
        {"motor_speed_rad_s", "motor_speed_rad_s"},
        {"pwm_us", "pwm_us"},
        {"ref_angle_rad", "ref_angle_rad"},
        {"valid", "valid"},
    };

    default_opt_.include_header_comments = true;
    default_opt_.decimals = 6;
    default_opt_.columns = {
        "Domain",
        "t_rel_s",
        "arm_angle_rad",
        "motor_speed_rad_s",
        "pwm_us",
        "ref_angle_rad",
        "valid"
    };

    setWindowTitle("Export Preview");
    setModal(true);
    resize(980, 560);

    buildUi_();
}

void ExportPreviewDialog::setDefaultOptions(const prop_arm::models::CsvExportOptions& opt) {
    default_opt_ = opt;

    if (include_header_comments_) {
        include_header_comments_->setChecked(default_opt_.include_header_comments);
    }
    if (decimals_) {
        decimals_->setValue(static_cast<double>(std::clamp(default_opt_.decimals, 0, 12)));
    }

    if (columns_list_) {
        // Default all unchecked then enable those in opt.columns
        for (int i = 0; i < columns_list_->count(); ++i) {
            auto* it = columns_list_->item(i);
            it->setCheckState(Qt::Unchecked);
        }
        for (const auto& col : default_opt_.columns) {
            setCheckboxStateByKey_(QString::fromStdString(col), true);
        }
    }

    onOptionsChanged_();
}

void ExportPreviewDialog::setSamples(const QVector<prop_arm::models::TelemetrySample>& samples) {
    samples_ = samples;
    rebuildModel_();
}

void ExportPreviewDialog::buildUi_() {
    title_ = new QLabel("Telemetry Export Preview", this);
    title_->setObjectName("title");

    // Table
    table_ = new QTableView(this);
    table_->setObjectName("panel");
    table_->setAlternatingRowColors(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);

    model_ = new QStandardItemModel(this);
    table_->setModel(model_);

    // Options panel
    auto* options_panel = buildOptionsPanel_();

    // Buttons
    save_ = new QPushButton("Save CSV", this);
    close_ = new QPushButton("Close", this);

    connect(save_, &QPushButton::clicked, this, &ExportPreviewDialog::onSaveCsv_);
    connect(close_, &QPushButton::clicked, this, &QDialog::accept);

    auto* btns = new QHBoxLayout();
    btns->addStretch(1);
    btns->addWidget(save_);
    btns->addWidget(close_);

    // Layout: options on left, preview on right
    auto* center = new QHBoxLayout();
    center->setSpacing(12);
    center->addWidget(options_panel);
    center->addWidget(table_, 1);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(10);
    root->addWidget(title_);
    root->addLayout(center, 1);
    root->addLayout(btns);

    // Apply defaults to widgets
    setDefaultOptions(default_opt_);
}

QWidget* ExportPreviewDialog::buildOptionsPanel_() {
    auto* w = new QWidget(this);
    w->setObjectName("optionsPanel");

    options_group_ = new QGroupBox("Export Options", w);
    options_group_->setObjectName("controlGroup");

    columns_list_ = new QListWidget(options_group_);
    columns_list_->setObjectName("panel");
    columns_list_->setMinimumWidth(280);

    for (const auto& c : columns_) {
        auto* it = new QListWidgetItem(c.label, columns_list_);
        it->setData(Qt::UserRole, c.key);
        it->setFlags(it->flags() | Qt::ItemIsUserCheckable);
        it->setCheckState(Qt::Unchecked);
        columns_list_->addItem(it);
    }

    include_header_comments_ = new QCheckBox("Include header comments", options_group_);
    include_header_comments_->setChecked(true);

    decimals_ = new QDoubleSpinBox(options_group_);
    decimals_->setDecimals(0);
    decimals_->setRange(0, 12);
    decimals_->setValue(6);
    decimals_->setSuffix(" decimals");

    auto* format_row = new QHBoxLayout();
    format_row->addWidget(include_header_comments_, 1);
    format_row->addWidget(decimals_, 0);

    // Filters
    auto* filters_box = new QGroupBox("Filters (preview + export)", options_group_);
    filters_box->setObjectName("controlSubGroup");

    filter_real_ = new QCheckBox("Real", filters_box);
    filter_sim_ = new QCheckBox("Sim", filters_box);
    show_valid_only_ = new QCheckBox("Valid only", filters_box);

    filter_real_->setChecked(true);
    filter_sim_->setChecked(true);
    show_valid_only_->setChecked(true);

    auto* filt = new QVBoxLayout(filters_box);
    filt->setSpacing(6);
    filt->addWidget(filter_real_);
    filt->addWidget(filter_sim_);
    filt->addWidget(show_valid_only_);

    auto* opt = new QVBoxLayout(options_group_);
    opt->setContentsMargins(12, 12, 12, 12);
    opt->setSpacing(10);
    opt->addWidget(new QLabel("Columns:", options_group_));
    opt->addWidget(columns_list_, 1);
    opt->addLayout(format_row);
    opt->addWidget(filters_box);

    auto* root = new QVBoxLayout(w);
    root->setContentsMargins(0, 0, 0, 0);
    root->addWidget(options_group_);

    // Rebuild preview when options change
    connect(columns_list_, &QListWidget::itemChanged, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(include_header_comments_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(decimals_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ExportPreviewDialog::onOptionsChanged_);
    connect(filter_real_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(filter_sim_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);
    connect(show_valid_only_, &QCheckBox::toggled, this, &ExportPreviewDialog::onOptionsChanged_);

    return w;
}

void ExportPreviewDialog::onOptionsChanged_() {
    rebuildModel_();
}

bool ExportPreviewDialog::isColumnChecked_(const QString& key) const {
    if (!columns_list_) return false;
    for (int i = 0; i < columns_list_->count(); ++i) {
        const auto* it = columns_list_->item(i);
        if (it->data(Qt::UserRole).toString() == key) {
            return it->checkState() == Qt::Checked;
        }
    }
    return false;
}

void ExportPreviewDialog::setCheckboxStateByKey_(const QString& key, bool checked) {
    if (!columns_list_) return;
    for (int i = 0; i < columns_list_->count(); ++i) {
        auto* it = columns_list_->item(i);
        if (it->data(Qt::UserRole).toString() == key) {
            it->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
            return;
        }
    }
}

prop_arm::models::CsvExportOptions ExportPreviewDialog::collectOptions_() const {
    prop_arm::models::CsvExportOptions opt;
    opt.include_header_comments = include_header_comments_ ? include_header_comments_->isChecked() : true;

    const int dec = decimals_ ? static_cast<int>(decimals_->value()) : 6;
    opt.decimals = std::clamp(dec, 0, 12);

    opt.columns.clear();
    opt.columns.reserve(static_cast<std::size_t>(columns_.size()));

    for (const auto& c : columns_) {
        if (isColumnChecked_(c.key)) {
            opt.columns.push_back(c.key.toStdString());
        }
    }

    // Guard: must export at least one column
    if (opt.columns.empty()) {
        opt.columns.push_back("Domain");
        opt.columns.push_back("arm_angle_rad");
    }

    return opt;
}

std::vector<prop_arm::models::TelemetrySample>
ExportPreviewDialog::toStd_(const QVector<prop_arm::models::TelemetrySample>& v) const {
    std::vector<prop_arm::models::TelemetrySample> out;
    out.reserve(static_cast<std::size_t>(v.size()));
    for (const auto& s : v) out.push_back(s);
    return out;
}

void ExportPreviewDialog::rebuildModel_() {
    model_->clear();

    // Decide which columns are visible in the preview
    std::vector<QString> keys;
    std::vector<QString> labels;

    keys.reserve(columns_.size());
    labels.reserve(columns_.size());

    for (const auto& c : columns_) {
        if (isColumnChecked_(c.key)) {
            keys.push_back(c.key);
            labels.push_back(c.key);  // keep header consistent with CSV keys
        }
    }

    if (keys.empty()) {
        keys = { "Domain", "arm_angle_rad" };
        labels = { "Domain", "arm_angle_rad" };
    }

    QStringList header;
    for (const auto& h : labels) header << h;
    model_->setHorizontalHeaderLabels(header);

    // Filters
    const bool want_real = filter_real_ ? filter_real_->isChecked() : true;
    const bool want_sim  = filter_sim_  ? filter_sim_->isChecked()  : true;
    const bool valid_only = show_valid_only_ ? show_valid_only_->isChecked() : false;

    // Build rows
    int row = 0;
    model_->setRowCount(0);

    const int dec = decimals_ ? static_cast<int>(decimals_->value()) : 6;

    for (const auto& s : samples_) {
        if (valid_only && !s.valid) continue;
        if (s.domain == prop_arm::models::Domain::Real && !want_real) continue;
        if (s.domain == prop_arm::models::Domain::Sim  && !want_sim)  continue;

        model_->insertRow(row);

        for (int c = 0; c < static_cast<int>(keys.size()); ++c) {
            const auto& k = keys[static_cast<std::size_t>(c)];

            if (k == "Domain") {
                model_->setItem(row, c, mkItem(domainToUi(s.domain)));
            } else if (k == "t_rel_s") {
                // If you don't store relative time yet, keep it empty in preview.
                // DataExporter can still compute it if your exporter supports it.
                model_->setItem(row, c, mkItem(""));
            } else if (k == "arm_angle_rad") {
                model_->setItem(row, c, mkItem(QString::number(s.arm_angle_rad, 'f', dec)));
            } else if (k == "motor_speed_rad_s") {
                model_->setItem(row, c, mkItem(QString::number(s.motor_speed_rad_s, 'f', dec)));
            } else if (k == "pwm_us") {
                model_->setItem(row, c, mkItem(QString::number(s.pwm_us)));
            } else if (k == "ref_angle_rad") {
                model_->setItem(row, c, mkItem(QString::number(s.ref_angle_rad, 'f', dec)));
            } else if (k == "valid") {
                model_->setItem(row, c, mkItem(s.valid ? "true" : "false"));
            } else {
                model_->setItem(row, c, mkItem(""));
            }
        }

        ++row;

        // Keep preview fast
        if (row >= 3000) break;
    }

    table_->resizeColumnsToContents();
}

void ExportPreviewDialog::onSaveCsv_() {
    if (samples_.isEmpty()) {
        QMessageBox::warning(this, "Export", "No samples available.");
        return;
    }

    const QString suggested =
        default_filename_.isEmpty() ? QStringLiteral("telemetry.csv")
        : default_filename_;

    const QString path = QFileDialog::getSaveFileName(
                             this,
                             "Save CSV",
                             suggested,
                             "CSV Files (*.csv);;All Files (*)"
                         );
    if (path.isEmpty()) return;

    // Apply same filters to exported data as preview
    QVector<prop_arm::models::TelemetrySample> filtered;
    filtered.reserve(samples_.size());

    const bool want_real   = filter_real_ ? filter_real_->isChecked() : true;
    const bool want_sim    = filter_sim_ ? filter_sim_->isChecked() : true;
    const bool valid_only  = show_valid_only_ ? show_valid_only_->isChecked() : false;

    for (const auto& s : samples_) {
        if (valid_only && !s.valid) continue;
        if (s.domain == prop_arm::models::Domain::Real && !want_real) continue;
        if (s.domain == prop_arm::models::Domain::Sim  && !want_sim)  continue;
        filtered.push_back(s);
    }

    if (filtered.isEmpty()) {
        QMessageBox::warning(this, "Export", "No samples match the selected filters.");
        return;
    }

    const auto opt = collectOptions_();

    prop_arm::core::logging::DataExporter exporter;
    const bool ok = exporter.exportToCsv(
                        path.toStdString(),
                        toStd_(filtered),
                        opt
                    );

    if (!ok) {
        QMessageBox::critical(
            this,
            "Export Failed",
            QString("Failed to export data:\n%1").arg(QString::fromStdString(exporter.lastError()))
        );
        return;
    }

    QMessageBox::information(
        this,
        "Export Complete",
        QString("Saved %1 samples to:\n%2").arg(filtered.size()).arg(path)
    );
}

void ExportPreviewDialog::setDefaultFilename(const QString& name) {
    default_filename_ = name.trimmed();
}


}  // namespace prop_arm::ui::widgets
