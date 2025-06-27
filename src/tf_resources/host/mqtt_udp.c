/*
 * MQTT - UDP bridge
 * 
 * Data is encoded as strings for both MQTT messages and UDP payload.
 * The first word in the payload represents the topic or operation and
 * the rest of the payload represents data.
 * 
 * Possible operations (sent from UDP client) are:
 * [TOPIC topic/subtopic] - register a new topic
 * [PUBLISH topic/subtopic data... ]
 * 
 * Data (sent from the Internet) is encoded as:
 * [topic/subtopic data... ]
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <MQTTClient.h>

#define MQTT_CID_PREFIX		"mqttbridge"
#define MQTT_TIMEOUT		5000L
#define UDP_ADDRESS		"172.31.69.20"
#define UDP_CPORT		30168
#define UDP_SPORT		8888
#define UDP_BUFLEN		1024

struct sockaddr_in si_me, si_other;
int s, slen = sizeof(si_other), recv_len;
char recv_buf[UDP_BUFLEN];
char send_buf[UDP_BUFLEN];


void publish(MQTTClient client, char *topic, char *payload) {
	MQTTClient_message pubmsg = MQTTClient_message_initializer;
	pubmsg.payload = payload;
	pubmsg.payloadlen = strlen(pubmsg.payload);
	pubmsg.qos = 0;
	pubmsg.retained = 0;
	MQTTClient_deliveryToken token;
	MQTTClient_publishMessage(client, topic, &pubmsg, &token);
	MQTTClient_waitForCompletion(client, token, MQTT_TIMEOUT);
	
	printf("topic: %s, pub '%s'\n", topic, payload);
}

int on_message(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
	char *payload = message->payload;
	int size = 0;

	size = strlen(topicName) + strlen(payload) + 2;
	if (size >= UDP_BUFLEN)
		return 1;

	/* receive data from topic */
	printf("topic: %s, data '%s'\n", topicName, payload);
	
	/* send data to UDP client */
	strcpy(send_buf, topicName);
	strcat(send_buf, " ");
	strcat(send_buf, payload);
	if (sendto(s, send_buf, strlen(send_buf), 0, (struct sockaddr *) &si_other, slen) == -1) {
		printf("failed sending UDP data to client\n");
		
		return -1;
	}
	
	MQTTClient_freeMessage(&message);
	MQTTClient_free(topicName);
	
	return 1;
}

int main(int argc, char **argv) {
	int rc;
	char *topic, *str;
	char client_id[128];
	uint16_t id;
	
	if (argc != 2) {
		printf("Usage: %s server:port\n", argv[0]);
		
		return -1;
	}

	/* create a UDP socket */
	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
		return -1;
    
	/* zero out the structure */
	memset((char *) &si_me, 0, sizeof(si_me));
     	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(UDP_SPORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
     
	/* bind socket to port */
	if (bind(s, (struct sockaddr*)&si_me, sizeof(si_me)) == -1)
		return -1;
		
	/* configure client */
	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(UDP_CPORT);

	if (inet_aton(UDP_ADDRESS, &si_other.sin_addr) == 0)
		return -1;

	srand(getpid());
	id = rand() % 65536;
	sprintf(client_id, "%s%04x", MQTT_CID_PREFIX, id);
	
	/* create MQTT client context */
	MQTTClient client;
	MQTTClient_create(&client, argv[1], client_id, MQTTCLIENT_PERSISTENCE_NONE, NULL);
	MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
	
	/* configure callback for messages */
	MQTTClient_setCallbacks(client, NULL, NULL, on_message, NULL);

	if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
		printf("Failed to connect, return code %d\n", rc);
		exit(-1);
	}

	printf("Connected to %s, client id: %s\n", argv[1], client_id);

	for (;;) {
		memset(recv_buf, 0, UDP_BUFLEN);
		/* try to receive some data from the UDP client, this is a blocking call */
		if ((recv_len = recvfrom(s, recv_buf, UDP_BUFLEN - 2, 0, (struct sockaddr *) &si_other, (socklen_t *)&slen)) == -1) {
			printf("failed receiving UDP data from client\n");
			
			return -1;
		}
		
		/* parse data */
		topic = strtok(recv_buf, " ");
		if (topic == NULL) continue;
		
		/* new topic */
		if (!strcmp(topic, "TOPIC")) {
			str = strtok(NULL, "");
			MQTTClient_subscribe(client, str, 0);
			printf("subscribed to topic %s\n", str);
			
			continue;
		}
		
		/* publish data */
		if (!strcmp(topic, "PUBLISH")) {
			topic = strtok(NULL, " ");
			if (topic == NULL) continue;
			str = strtok(NULL, "");
			if (str == NULL) continue;
			publish(client, topic, str);
			
			continue;
		}
		
		str = strtok(NULL, "");
		printf("data: %s %s\n", topic, str);
	}

	MQTTClient_disconnect(client, 1000);
	MQTTClient_destroy(&client);
	
	return rc;
}
