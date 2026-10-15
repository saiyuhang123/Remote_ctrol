#include "adapters/business_client.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkReply>
#include <QNetworkRequest>

BusinessClient::BusinessClient(QObject *parent) : QObject(parent) {}
void BusinessClient::setBaseUrl(const QString &url) { baseUrl_ = url; while (baseUrl_.endsWith('/')) baseUrl_.chop(1); }
void BusinessClient::request(const QByteArray &method, const QString &path, const QByteArray &body) {
  QNetworkRequest request{QUrl(baseUrl_ + path)};
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  request.setTransferTimeout(5000);
  auto *reply = network_.sendCustomRequest(request, method, body);
  connect(reply, &QNetworkReply::finished, this, [this, reply, method] {
    const auto bytes = reply->readAll(); const bool ok = reply->error() == QNetworkReply::NoError;
    if (!ok) emit message("业务后端：" + reply->errorString());
    else if (method == "GET") {
      QJsonParseError error; auto doc = QJsonDocument::fromJson(bytes, &error);
      if (error.error == QJsonParseError::NoError && doc.isObject() && doc.object().value("items").isArray()) emit points(doc.object().value("items").toArray());
      else emit message("点位列表响应格式错误");
    } else { emit message("点位已保存到现有业务后端"); refreshPoints(); }
    reply->deleteLater();
  });
}
void BusinessClient::refreshPoints() { request("GET", "/api/points"); }
void BusinessClient::addPoint(const QString &name, double x, double y, double yaw) {
  request("POST", "/api/points", QJsonDocument(QJsonObject{{"name", name}, {"x", x}, {"y", y}, {"yaw", yaw}, {"actions", QJsonArray{}}}).toJson(QJsonDocument::Compact));
}
void BusinessClient::deletePoint(int id) { request("DELETE", "/api/points/" + QString::number(id)); }
