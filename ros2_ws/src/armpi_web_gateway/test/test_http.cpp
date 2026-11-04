// HTTP 형식 도우미 시험 + 실제 소켓 서버 왕복 1회
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "armpi_web_gateway/http.hpp"
#include "armpi_web_gateway/http_server.hpp"

using namespace armpi_web_gateway;

TEST(Http, ParseRequestHead) {
  const auto r = parse_request_head("POST /api/cmd?x=1 HTTP/1.1\r\nHost: 1.2.3.4:18765\r\nContent-Type: application/json\r\n"
                                    "X-A: 1\r\nx-a: 2\r\n\r\n");
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->method, "POST");
  EXPECT_EQ(r->path, "/api/cmd");
  EXPECT_EQ(r->query, "x=1");
  EXPECT_EQ(r->header("host"), "1.2.3.4:18765");
  EXPECT_EQ(r->header("x-a"), "1");  // 같은 이름은 첫 값
  EXPECT_FALSE(parse_request_head("garbage\r\n\r\n"));
  EXPECT_FALSE(parse_request_head("GET http://x/ HTTP/1.1\r\n\r\n"));
  EXPECT_FALSE(parse_request_head("GET / HTTP/1.1\r\nno colon\r\n\r\n"));
}

TEST(Http, SplitHostAndIp) {
  auto h = split_host("Example.LOCAL:18765");
  ASSERT_TRUE(h);
  EXPECT_EQ(h->first, "example.local");
  EXPECT_EQ(*h->second, 18765);
  h = split_host("[fe80::1]:8080");
  ASSERT_TRUE(h);
  EXPECT_EQ(h->first, "fe80::1");
  h = split_host("host");
  ASSERT_TRUE(h);
  EXPECT_FALSE(h->second.has_value());
  EXPECT_FALSE(split_host("host:abc"));
  EXPECT_FALSE(split_host("[::1"));
  EXPECT_TRUE(is_ip_literal("192.168.1.1"));
  EXPECT_TRUE(is_ip_literal("::1"));
  EXPECT_FALSE(is_ip_literal("armpi.local"));
}

TEST(Http, QueryParam) {
  EXPECT_EQ(query_param("screen=a%2Db&x=1", "screen"), "a-b");
  EXPECT_EQ(query_param("a=1&screen=x+y", "screen"), "x y");
  EXPECT_FALSE(query_param("screen=", "screen"));
  EXPECT_FALSE(query_param("", "screen"));
}

TEST(Http, ServerRoundTrip) {
  HttpServer s("127.0.0.1", 0, [](const HttpRequest& r) {
    HttpResponse res;
    res.body = r.method + " " + r.path + " " + r.body + " " + r.client_addr;
    return res;
  });
  std::string err;
  ASSERT_TRUE(s.start(&err)) << err;
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(static_cast<uint16_t>(s.port()));
  ::inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
  ASSERT_EQ(::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
  const std::string req = "POST /p HTTP/1.1\r\nHost: x\r\nContent-Length: 5\r\n\r\nhello";
  ASSERT_EQ(::send(fd, req.data(), req.size(), 0), static_cast<ssize_t>(req.size()));
  std::string got;
  char buf[1024];
  ssize_t n;
  while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0) got.append(buf, static_cast<size_t>(n));
  ::close(fd);
  s.stop();
  EXPECT_EQ(got.rfind("HTTP/1.1 200 OK\r\n", 0), 0u);
  EXPECT_NE(got.find("Connection: close"), std::string::npos);
  EXPECT_NE(got.find("POST /p hello 127.0.0.1"), std::string::npos);
}
