#include <spot/auth.h>

const std::string AUTH_CLIENT_NAME = "auth";
const static std::string AUTHORITY = "auth.spot.robot";
const static std::string TOKEN = "";

AuthClient::AuthClient() : BaseClient(AUTH_CLIENT_NAME, AUTHORITY, TOKEN) {}

GetAuthTokenResponse AuthClient::auth(const std::string &user, const std::string &pass) {
	GetAuthTokenRequest request;
	assembleRequestHeader<GetAuthTokenRequest>(&request);
	request.set_username(user);
    request.set_password(pass);

	return call<GetAuthTokenRequest, GetAuthTokenResponse>(request, &AuthService::Stub::GetAuthToken);
}

GetAuthTokenResponse AuthClient::authAsync(const std::string &user, const std::string &pass) {
	GetAuthTokenRequest request;
	assembleRequestHeader<GetAuthTokenRequest>(&request);
	request.set_username(user);
    request.set_password(pass);

	return callAsync<GetAuthTokenRequest, GetAuthTokenResponse>(request, &AuthService::Stub::AsyncGetAuthToken);
}

GetAuthTokenResponse AuthClient::authWithToken(const std::string &token) {
	GetAuthTokenRequest request;
	assembleRequestHeader<GetAuthTokenRequest>(&request);
	request.set_token(token);

	return call<GetAuthTokenRequest, GetAuthTokenResponse>(request, &AuthService::Stub::GetAuthToken);	
}

GetAuthTokenResponse AuthClient::authWithTokenAsync(const std::string &token) {
	GetAuthTokenRequest request;
	assembleRequestHeader<GetAuthTokenRequest>(&request);
	request.set_token(token);

	return callAsync<GetAuthTokenRequest, GetAuthTokenResponse>(request, &AuthService::Stub::AsyncGetAuthToken);
}

std::string AuthClient::authenticate(const std::string &username, const std::string &password, bool async) {
	return async ? authAsync(username, password).token() : auth(username, password).token();
}


std::string AuthClient::authenticateWithToken(const std::string &token, bool async) {
	return async ? authWithTokenAsync(token).token() : authWithTokenAsync(token).token();
}