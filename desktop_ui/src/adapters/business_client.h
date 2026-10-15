#pragma once
#include <QJsonArray>
#include <QNetworkAccessManager>
#include <QObject>

class BusinessClient : public QObject {
  Q_OBJECT
public:
  explicit BusinessClient(QObject *parent = nullptr);
  void setBaseUrl(const QString &url);
  void refreshPoints();
  void addPoint(const QString &name, double x, double y, double yaw);
  void deletePoint(int id);
signals:
  void points(const QJsonArray &items);
  void message(const QString &text);
private:
  void request(const QByteArray &method, const QString &path, const QByteArray &body = {});
  QNetworkAccessManager network_;
  QString baseUrl_;
};
