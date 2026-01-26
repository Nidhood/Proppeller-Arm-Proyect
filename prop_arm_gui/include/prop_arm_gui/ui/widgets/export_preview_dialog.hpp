#pragma once

#include <QDialog>
#include <QVector>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "prop_arm_gui/core/models/csv_export_options.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"

QT_BEGIN_NAMESPACE
class QCheckBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QStandardItemModel;
class QTableView;
QT_END_NAMESPACE

namespace prop_arm::ui::widgets {

class ExportPreviewDialog final : public QDialog {
    Q_OBJECT

public:
    explicit ExportPreviewDialog(QWidget* parent = nullptr);
    void setSamples(const QVector<prop_arm::models::TelemetrySample>& samples);
    void setDefaultOptions(const prop_arm::models::CsvExportOptions& opt);
    void setDefaultFilename(const QString& name);


private slots:
    void onOptionsChanged_();
    void onSaveCsv_();

private:
    struct ColumnDef {
        QString key;
        QString label;
    };

private:
    void buildUi_();
    QWidget* buildOptionsPanel_();
    void rebuildModel_();
    prop_arm::models::CsvExportOptions collectOptions_() const;
    std::vector<prop_arm::models::TelemetrySample> toStd_(const QVector<prop_arm::models::TelemetrySample>& v) const;

    void setCheckboxStateByKey_(const QString& key, bool checked);
    bool isColumnChecked_(const QString& key) const;

private:
    QLabel* title_{nullptr};

    // Preview table
    QTableView* table_{nullptr};
    QStandardItemModel* model_{nullptr};
    QString default_filename_;

    // Options widgets
    QGroupBox* options_group_{nullptr};
    QListWidget* columns_list_{nullptr};
    QCheckBox* include_header_comments_{nullptr};
    QDoubleSpinBox* decimals_{nullptr};

    QCheckBox* filter_real_{nullptr};
    QCheckBox* filter_sim_{nullptr};
    QCheckBox* show_valid_only_{nullptr};

    // Actions
    QPushButton* save_{nullptr};
    QPushButton* close_{nullptr};

    QVector<prop_arm::models::TelemetrySample> samples_;

    std::vector<ColumnDef> columns_;
    prop_arm::models::CsvExportOptions default_opt_;
};

}  // namespace prop_arm::ui::widgets
