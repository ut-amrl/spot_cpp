/*
  directory.h: includes client for communication with the directory service 
*/

#ifndef DIRECTORY_H
#define DIRECTORY_H

#include <spot/clients/base.h>
#include "bosdyn/api/directory_service.grpc.pb.h"

using bosdyn::api::DirectoryService;
using bosdyn::api::ListServiceEntriesResponse;
using bosdyn::api::ListServiceEntriesRequest;
using bosdyn::api::GetServiceEntryRequest;
using bosdyn::api::GetServiceEntryResponse;

const extern std::string DIRECTORY_CLIENT_NAME;

namespace ClientLayer {
    /*
        class ServiceEntry: holds service entries returned from the directory service
    */
    class ServiceEntry {
    public:
        ServiceEntry(const std::string &name, const std::string &type, const std::string &authority) :
                _name(name),
                _type(type),
                _authority(authority) {}

        /* Accessors */
        const std::string getName() const { return _name; }
        const std::string getType() const { return _type; }
        const std::string getAuthority() const { return _authority; }
    private:
        const std::string _name;
        const std::string _type;
        const std::string _authority;
    };

	class DirectoryClient : public BaseClient<DirectoryService> {
	public:
		DirectoryClient(const std::string &token);

		// std::list<ServiceEntry> listServiceEntries(bool async); // idea for containing grpc calls

		ListServiceEntriesResponse list();
		ListServiceEntriesResponse listAsync();

		// TODO: figure out way to interface these methods which return gRPC objects
		GetServiceEntryResponse getEntry(std::string serviceName);
		GetServiceEntryResponse getEntryAsync(std::string serviceName);
	};

};


#endif