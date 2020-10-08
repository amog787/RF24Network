/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
#include "RF24Network_config.h"
#include "RF24.h"
#include "RF24Network.h"

static UART_HandleTypeDef nrf24_huart;
uint16_t RF24NetworkHeader::next_id = 1;

/******************************************************************/
#if !defined (DUAL_HEAD_RADIO)
RF24Network::RF24Network( RF24& _radio ): radio(_radio), next_frame(frame_queue)
{
  #if !defined ( DISABLE_FRAGMENTATION )
  frag_queue.message_buffer=&frag_queue_message_buffer[0];
  frag_ptr = &frag_queue;
  #endif
  networkFlags=0; returnSysMsgs=0; multicastRelay=0;
}
#else
RF24Network::RF24Network( RF24& _radio, RF24& _radio1 ): radio(_radio), radio1(_radio1), next_frame(frame_queue)
{
  #if !defined ( DISABLE_FRAGMENTATION )
  frag_queue.message_buffer=&frag_queue_message_buffer[0];
  frag_ptr = &frag_queue;
  #endif
  networkFlags=0; returnSysMsgs=0; multicastRelay=0;
}
#endif
/******************************************************************/

void RF24Network::begin(uint8_t _channel, uint16_t _node_address )
{
  if (! is_valid_address(_node_address) )
    return;

  node_address = _node_address;

  if ( ! radio.isValid() ){
    return;
  }

  // Set up the radio the way we want it to look
  if(_channel != USE_CURRENT_CHANNEL){
    radio.setChannel(_channel);
  }
  //radio.enableDynamicAck();
  radio.setAutoAck(1);
  radio.setAutoAck(0,0);

  #if defined (ENABLE_DYNAMIC_PAYLOADS)
  radio.enableDynamicPayloads();
  #endif

  // Use different retry periods to reduce data collisions
  uint8_t retryVar = (((node_address % 6)+1) *2) + 3;
  radio.setRetries(retryVar, 5); // max about 85ms per attempt
  txTimeout = 25;
  routeTimeout = txTimeout*3; // Adjust for max delay per node within a single chain

#if defined (DUAL_HEAD_RADIO)
  radio1.setChannel(_channel);
  radio1.enableDynamicAck();
  radio1.enableDynamicPayloads();
#endif

  // Setup our address helper cache
  setup_address();

  // Open up all listening pipes
  uint8_t i = 6;
  while (i--){
	  if(i!=0) //Why? needs more debugging
		  radio.openReadingPipe(i,pipe_address(_node_address,i));
  }

  radio.startListening();
}

/******************************************************************/

#if defined ENABLE_NETWORK_STATS
void RF24Network::failures(uint32_t *_fails, uint32_t *_ok){
	*_fails = nFails;
	*_ok = nOK;
}
#endif

/******************************************************************/

uint8_t RF24Network::update(void)
{
  // if there is data ready
  uint8_t pipe_num;
  uint8_t returnVal = 0;

  uint32_t timeout = HAL_GetTick();

  while ( radio.isValid() && radio.available(&pipe_num) ){
    if(HAL_GetTick() - timeout > 1000){
      #if defined FAILURE_HANDLING
        radio.failureDetected = 1;
      #endif
      break;
    }
    #if defined (ENABLE_DYNAMIC_PAYLOADS) && !defined (XMEGA_D3)
      frame_size = radio.getDynamicPayloadSize();
    #else
      frame_size=MAX_FRAME_SIZE;
    #endif

    // Fetch the payload, and see if this was the last one.
    radio.read( frame_buffer, frame_size );

    // Read the beginning of the frame as the header
    RF24NetworkHeader *header = (RF24NetworkHeader*)(&frame_buffer);

    // Throw it away if it's not a valid address or too small
    if ( frame_size < sizeof(RF24NetworkHeader) || !is_valid_address(header->to_node) || !is_valid_address(header->from_node) ){
      continue;
    }

      //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Received on %u %s\n\r"),HAL_GetTick(),pipe_num,header->toString()));
      //IF_SERIAL_DEBUG(const uint16_t* i = reinterpret_cast<const uint16_t*>(frame_buffer + sizeof(RF24NetworkHeader));printf_P(PSTR("%lu: NET message %04x\n\r"),HAL_GetTick(),*i));

    returnVal = header->type;
    // Is this for us?
    if ( header->to_node == node_address   ){

      if(header->type == NETWORK_PING){
        continue;
      }
      if(header->type == NETWORK_ADDR_RESPONSE ){
        uint16_t requester = NETWORK_DEFAULT_ADDRESS;
        if(requester != node_address){
          header->to_node = requester;
          write(header->to_node,USER_TX_TO_PHYSICAL_ADDRESS);
          continue;
        }
      }
      if(header->type == NETWORK_REQ_ADDRESS && node_address){
        header->from_node = node_address;
        header->to_node = 0;
        write(header->to_node,TX_NORMAL);
        continue;
      }
      if( (returnSysMsgs && header->type > 127) || header->type == NETWORK_ACK ){
        ////IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%lu MAC: System payload rcvd %d\n"),HAL_GetTick(),returnVal); );
        if( header->type != NETWORK_FIRST_FRAGMENT && header->type != NETWORK_MORE_FRAGMENTS && header->type != EXTERNAL_DATA_TYPE && header->type!= NETWORK_LAST_FRAGMENT){
          return returnVal;
        }
      }

      if( enqueue(header) == 2 ){ //External data received
        #if defined (SERIAL_DEBUG_MINIMAL)
          printf("ret ext\n");
        #endif
        return EXTERNAL_DATA_TYPE;
      }

    }else{
    #if defined	(RF24NetworkMulticast)

      if( header->to_node == 0100){
        if(header->type == NETWORK_POLL  ){
          returnVal = 0;
          if( !(networkFlags & FLAG_NO_POLL) && node_address != NETWORK_DEFAULT_ADDRESS ){
            header->to_node = header->from_node;
            header->from_node = node_address;
            HAL_Delay(parent_pipe);
            write(header->to_node,USER_TX_TO_PHYSICAL_ADDRESS);
          }
          continue;
        }

        uint8_t val = enqueue(header);

        if(multicastRelay){
          ////IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%u MAC: FWD multicast frame from 0%o to level %u\n"),HAL_GetTick(),header->from_node,multicast_level+1); );
          if ((node_address >> 3) != 0) {
            // for all but the first level of nodes, those not directly connected to the master, we add the total delay per level
        	  HAL_Delay(0.600*4);
          }
          HAL_Delay((node_address % 4)*0.600);
          write(levelToAddress(multicast_level)<<3,4);
        }
        if( val == 2 ){ //External data received
          return EXTERNAL_DATA_TYPE;
        }

      }else{
        if(node_address != NETWORK_DEFAULT_ADDRESS){
          write(header->to_node,1);	//Send it on, indicate it is a routed payload
          returnVal = 0;
        }
      }
    #else
      if(node_address != NETWORK_DEFAULT_ADDRESS){
        write(header->to_node,1);	//Send it on, indicate it is a routed payload
        returnVal = 0;
      }
    #endif
    }

  } // radio.available()
  return returnVal;
}

/******************************************************************/
/******************************************************************/

uint8_t RF24Network::enqueue(RF24NetworkHeader* header)
{
  bool result = false;
  uint16_t message_size = frame_size - sizeof(RF24NetworkHeader);

  //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Enqueue @%x "),HAL_GetTick(),next_frame-frame_queue));

#if !defined ( DISABLE_FRAGMENTATION )

  bool isFragment = header->type == NETWORK_FIRST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS || header->type == NETWORK_LAST_FRAGMENT;

  if(isFragment){

    if(header->type == NETWORK_FIRST_FRAGMENT){

		memcpy(&frag_queue,&frame_buffer,sizeof(RF24NetworkHeader));
		memcpy(frag_queue.message_buffer,frame_buffer+sizeof(RF24NetworkHeader),message_size);

        ////IF_SERIAL_DEBUG_FRAGMENTATION( Serial.print(F("queue first, total frags ")); Serial.println(header->reserved); );
        //Store the total size of the stored frame in message_size
        frag_queue.message_size = message_size;
        --frag_queue.header.reserved;
        ////IF_SERIAL_DEBUG_FRAGMENTATION_L2(  for(int i=0; i<frag_queue.message_size;i++){  Serial.println(frag_queue.message_buffer[i],HEX);  } );
        return true;

    }else // NETWORK_MORE_FRAGMENTS
    if(header->type == NETWORK_LAST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS){

        if(frag_queue.message_size + message_size > MAX_PAYLOAD_SIZE){
          #if defined (SERIAL_DEBUG_FRAGMENTATION) || defined (SERIAL_DEBUG_MINIMAL)
          Serial.print(F("Drop frag ")); Serial.print(header->reserved);
          Serial.println(F(" Size exceeds max"));
          #endif
          frag_queue.header.reserved=0;
          return false;
        }
        if(  frag_queue.header.reserved == 0 || (header->type != NETWORK_LAST_FRAGMENT && header->reserved != frag_queue.header.reserved ) || frag_queue.header.id != header->id ){
          #if defined (SERIAL_DEBUG_FRAGMENTATION) || defined (SERIAL_DEBUG_MINIMAL)
            Serial.print(F("Drop frag ")); Serial.print(header->reserved);
            Serial.println(F(" Out of order "));
          #endif
          return false;
        }

        memcpy(frag_queue.message_buffer+frag_queue.message_size,frame_buffer+sizeof(RF24NetworkHeader),message_size);
        frag_queue.message_size += message_size;

        if(header->type != NETWORK_LAST_FRAGMENT){
          --frag_queue.header.reserved;
          return true;
        }
        frag_queue.header.reserved = 0;
        frag_queue.header.type = header->reserved;

        ////IF_SERIAL_DEBUG_FRAGMENTATION( printf_P(PSTR("fq 3: %d\n"),frag_queue.message_size); );
        ////IF_SERIAL_DEBUG_FRAGMENTATION_L2(for(int i=0; i< frag_queue.message_size;i++){ Serial.println(frag_queue.message_buffer[i],HEX); }  );

        //Frame assembly complete, copy to main buffer if OK
        if(frag_queue.header.type == EXTERNAL_DATA_TYPE){
          return 2;
        }
        #if defined (DISABLE_USER_PAYLOADS)
          return 0;
        #endif

        if( (uint16_t)(MAX_PAYLOAD_SIZE) - (next_frame-frame_queue) >= frag_queue.message_size){
          memcpy(next_frame,&frag_queue,10);
          memcpy(next_frame+10,frag_queue.message_buffer,frag_queue.message_size);
          next_frame += (10+frag_queue.message_size);
          #if !defined(ARDUINO_ARCH_AVR)
          if(uint8_t padding = (frag_queue.message_size+10)%4){
            next_frame += 4 - padding;
          }
          #endif
          ////IF_SERIAL_DEBUG_FRAGMENTATION( printf_P(PSTR("enq size %d\n"),frag_queue.message_size); );
          return true;
        }
        ////IF_SERIAL_DEBUG_FRAGMENTATION( printf_P(PSTR("Drop frag payload, queue full\n")); );
        return false;
    }//If more or last fragments

  }else //else is not a fragment
#endif // End fragmentation enabled

  // Copy the current frame into the frame queue
#if !defined( DISABLE_FRAGMENTATION )
    if(header->type == EXTERNAL_DATA_TYPE){
      memcpy(&frag_queue,&frame_buffer,8);
      frag_queue.message_buffer = frame_buffer+sizeof(RF24NetworkHeader);
      frag_queue.message_size = message_size;
      return 2;
    }
#endif
#if defined (DISABLE_USER_PAYLOADS)
    return 0;
  }
#else
  if(message_size + (next_frame-frame_queue) <= MAIN_BUFFER_SIZE){
	memcpy(next_frame,&frame_buffer,8);
    memcpy(next_frame+8,&message_size,2);
	memcpy(next_frame+10,frame_buffer+8,message_size);

	//////IF_SERIAL_DEBUG_FRAGMENTATION( for(int i=0; i<message_size;i++){ Serial.print(next_frame[i],HEX); Serial.print(" : "); } Serial.println(""); );

	next_frame += (message_size + 10);
    #if !defined(ARDUINO_ARCH_AVR)
    if(uint8_t padding = (message_size+10)%4){
      next_frame += 4 - padding;
    }
    #endif
  //////IF_SERIAL_DEBUG_FRAGMENTATION( Serial.print("Enq "); Serial.println(next_frame-frame_queue); );//printf_P(PSTR("enq %d\n"),next_frame-frame_queue); );

    result = true;
  }else{
    result = false;
    //IF_SERIAL_DEBUG(printf_P(PSTR("NET **Drop Payload** Buffer Full")));
  }
  return result;
}
#endif //USER_PAYLOADS_ENABLED

/******************************************************************/

bool RF24Network::available(void)
{
  // Are there frames on the queue for us?
  return (next_frame > frame_queue);
}

/******************************************************************/

uint16_t RF24Network::parent() const
{
  if ( node_address == 0 )
    return -1;
  else
    return parent_node;
}

/******************************************************************/

uint16_t RF24Network::peek(RF24NetworkHeader& header)
{
  if (available())
  {
	RF24NetworkFrame *frame = (RF24NetworkFrame*)(frame_queue);
	memcpy(&header,&frame->header,sizeof(RF24NetworkHeader));
    uint16_t msg_size;
    memcpy(&msg_size,frame_queue+8,2);
    return msg_size;
  }
  return 0;
}

/******************************************************************/

void RF24Network::peek(RF24NetworkHeader& header, void* message, uint16_t maxlen)
{
  if(available())
  {
    memcpy(&header, frame_queue, 8); //Copy the header
    if(maxlen > 0)
    {
      memcpy(message, frame_queue + 10, maxlen); //Copy the message
    }
  }
}

/******************************************************************/

uint16_t RF24Network::read(RF24NetworkHeader& header,void* message, uint16_t maxlen)
{
  uint16_t bufsize = 0;

  if (available())
  {

	memcpy(&header,frame_queue,8);
    memcpy(&bufsize,frame_queue+8,2);

    if (maxlen > 0)
    {
		maxlen = rf24_min(maxlen,bufsize);
		memcpy(message,frame_queue+10,maxlen);
	    //IF_SERIAL_DEBUG(printf("%lu: NET message size %d\n",HAL_GetTick(),bufsize););


	//IF_SERIAL_DEBUG( uint16_t len = maxlen; printf_P(PSTR("%lu: NET r message "),HAL_GetTick());const uint8_t* charPtr = reinterpret_cast<const uint8_t*>(message);while(len--){ printf("%02x ",charPtr[len]);} printf_P(PSTR("\n\r") ) );

    }
	next_frame-=bufsize+10;
    uint8_t padding = 0;
    #if !defined(ARDUINO_ARCH_AVR)
    if( (padding = (bufsize+10)%4) ){
      padding = 4-padding;
      next_frame -= padding;
    }
    #endif
    memmove(frame_queue,frame_queue+bufsize+10+padding,sizeof(frame_queue)- bufsize);
	////IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Received %s\n\r"),HAL_GetTick(),header.toString()));
  }

  return bufsize;
}


#if defined RF24NetworkMulticast
/******************************************************************/
bool RF24Network::multicast(RF24NetworkHeader& header,const void* message, uint16_t len, uint8_t level){
	// Fill out the header
  header.to_node = 0100;
  header.from_node = node_address;
  return write(header, message, len, levelToAddress(level));
}
#endif

/******************************************************************/
bool RF24Network::write(RF24NetworkHeader& header,const void* message, uint16_t len){
	return write(header,message,len,070);
}
/******************************************************************/
bool RF24Network::write(RF24NetworkHeader& header,const void* message, uint16_t len, uint16_t writeDirect){

#if defined (DISABLE_FRAGMENTATION)
    frame_size = rf24_min(len+sizeof(RF24NetworkHeader),MAX_FRAME_SIZE);
	return _write(header,message,rf24_min(len,max_frame_payload_size),writeDirect);
#else
  if(len <= max_frame_payload_size){
    //Normal Write (Un-Fragmented)
	frame_size = len + sizeof(RF24NetworkHeader);
    if(_write(header,message,len,writeDirect)){
      return 1;
    }
    return 0;
  }
  //Check payload size
  if (len > MAX_PAYLOAD_SIZE) {
    //IF_SERIAL_DEBUG(printf("%u: NET write message failed. Given 'len' %d is bigger than the MAX Payload size %i\n\r",HAL_GetTick(),len,MAX_PAYLOAD_SIZE););
    return false;
  }

  //Divide the message payload into chunks of max_frame_payload_size
  uint8_t fragment_id = (len % max_frame_payload_size != 0) + ((len ) / max_frame_payload_size);  //the number of fragments to send = ceil(len/max_frame_payload_size)

  uint8_t msgCount = 0;

  ////IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG Total message fragments %d\n\r",HAL_GetTick(),fragment_id););

  if(header.to_node != 0100){
    networkFlags |= FLAG_FAST_FRAG;
	#if !defined (DUAL_HEAD_RADIO)
	radio.stopListening();
	#endif
  }

  uint8_t retriesPerFrag = 0;
  uint8_t type = header.type;
  bool ok = 0;

  while (fragment_id > 0) {

    //Copy and fill out the header
    //RF24NetworkHeader fragmentHeader = header;
   header.reserved = fragment_id;

    if (fragment_id == 1) {
      header.type = NETWORK_LAST_FRAGMENT;  //Set the last fragment flag to indicate the last fragment
      header.reserved = type; //The reserved field is used to transmit the header type
    } else {
      if (msgCount == 0) {
        header.type = NETWORK_FIRST_FRAGMENT;
      }else{
        header.type = NETWORK_MORE_FRAGMENTS; //Set the more fragments flag to indicate a fragmented frame
      }
    }

    uint16_t offset = msgCount*max_frame_payload_size;
	uint16_t fragmentLen = rf24_min((uint16_t)(len-offset),max_frame_payload_size);

    //Try to send the payload chunk with the copied header
    frame_size = sizeof(RF24NetworkHeader)+fragmentLen;
	ok = _write(header,((char *)message)+offset,fragmentLen,writeDirect);

	if (!ok) {
	   HAL_Delay(2);
	   ++retriesPerFrag;
	}else{
	  retriesPerFrag = 0;
	  fragment_id--;
      msgCount++;
	}

    //if(writeDirect != 070){ HAL_Delay(2); } //Delay 2ms between sending multicast payloads

	if (!ok && retriesPerFrag >= 3) {
        ////IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG TX with fragmentID '%d' failed after %d fragments. Abort.\n\r",HAL_GetTick(),fragment_id,msgCount););
		break;
    }


    //Message was successful sent
    #if defined SERIAL_DEBUG_FRAGMENTATION_L2
	  printf("%lu: FRG message transmission with fragmentID '%d' sucessfull.\n\r",HAL_GetTick(),fragment_id);
	#endif

  }
  header.type = type;
  #if !defined (DUAL_HEAD_RADIO)
  if(networkFlags & FLAG_FAST_FRAG){
    ok = radio.txStandBy(txTimeout);
    radio.startListening();
    radio.setAutoAck(0,0);
  }
  networkFlags &= ~FLAG_FAST_FRAG;
  #endif


  //Return true if all the chunks where sent successfully

  ////IF_SERIAL_DEBUG_FRAGMENTATION(printf("%u: FRG total message fragments sent %i. \n",HAL_GetTick(),msgCount); );
  if(!ok || fragment_id > 0){
	return false;
  }
  return true;

#endif //Fragmentation enabled
}
/******************************************************************/

bool RF24Network::_write(RF24NetworkHeader& header,const void* message, uint16_t len, uint16_t writeDirect)
{
  // Fill out the header
  header.from_node = node_address;

  // Build the full frame to send
  memcpy(frame_buffer,&header,sizeof(RF24NetworkHeader));

    //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n\r"),HAL_GetTick(),header.toString()));

  if (len){

    memcpy(frame_buffer + sizeof(RF24NetworkHeader),message,len);

	//IF_SERIAL_DEBUG(uint16_t tmpLen = len;printf_P(PSTR("%lu: NET message "),HAL_GetTick());const uint8_t* charPtr = reinterpret_cast<const uint8_t*>(message);while(tmpLen--){ printf("%02x ",charPtr[tmpLen]);} printf_P(PSTR("\n\r") ) );

  }

  // If the user is trying to send it to himself
  /*if ( header.to_node == node_address ){
	#if defined (RF24_LINUX)
	  RF24NetworkFrame frame = RF24NetworkFrame(header,message,rf24_min(MAX_FRAME_SIZE-sizeof(RF24NetworkHeader),len));
	#else
      RF24NetworkFrame frame(header,len);
    #endif
	// Just queue it in the received queue
    return enqueue(frame);
  }*/
    // Otherwise send it out over the air


	if(writeDirect != 070){
		uint8_t sendType = USER_TX_TO_LOGICAL_ADDRESS; // Payload is multicast to the first node, and routed normally to the next

		if(header.to_node == 0100){
		  sendType = USER_TX_MULTICAST;
		}
		if(header.to_node == writeDirect){
		  sendType = USER_TX_TO_PHYSICAL_ADDRESS; // Payload is multicast to the first node, which is the recipient
		}
		return write(writeDirect,sendType);
	}
	return write(header.to_node,TX_NORMAL);

}

/******************************************************************/

bool RF24Network::write(uint16_t to_node, uint8_t directTo)  // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route
{
  bool ok = false;
  bool isAckType = false;
  if(frame_buffer[6] > 64 && frame_buffer[6] < 192 ){ isAckType=true; }

  /*if( ( (frame_buffer[7] % 2) && frame_buffer[6] == NETWORK_MORE_FRAGMENTS) ){
	isAckType = 0;
  }*/

  // Throw it away if it's not a valid address
  if ( !is_valid_address(to_node) )
    return false;

  //Load info into our conversion structure, and get the converted address info
  logicalToPhysicalStruct conversion = { to_node,directTo,0};
  logicalToPhysicalAddress(&conversion);


  //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r"),HAL_GetTick(),to_node,conversion.send_node,conversion.send_pipe));

  /**Write it*/
  if( directTo == TX_ROUTED && conversion.send_node == to_node && isAckType){
    HAL_Delay(2);
  }
  ok=write_to_pipe(conversion.send_node, conversion.send_pipe, conversion.multicast);

    if(!ok){
	////IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%lu: MAC Send fail to 0%o via 0%o on pipe %x\n\r"),HAL_GetTick(),to_node,conversion.send_node,conversion.send_pipe););
	}


	if( directTo == TX_ROUTED && ok && conversion.send_node == to_node && isAckType){

			RF24NetworkHeader* header = (RF24NetworkHeader*)&frame_buffer;
			header->type = NETWORK_ACK;				    // Set the payload type to NETWORK_ACK
			header->to_node = header->from_node;          // Change the 'to' address to the 'from' address

			conversion.send_node = header->from_node;
			conversion.send_pipe = TX_ROUTED;
			conversion.multicast = 0;
			logicalToPhysicalAddress(&conversion);

			//Write the data using the resulting physical address
			frame_size = sizeof(RF24NetworkHeader);
			write_to_pipe(conversion.send_node, conversion.send_pipe, conversion.multicast);

			//dynLen=0;
			    ////IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%lu MAC: Route OK to 0%o ACK sent to 0%o\n"),HAL_GetTick(),to_node,header->from_node); );

	}



	if( ok && conversion.send_node != to_node && (directTo==0 || directTo==3) && isAckType){
	    #if !defined (DUAL_HEAD_RADIO)
          // Now, continue listening
		  if(networkFlags & FLAG_FAST_FRAG){
			 radio.txStandBy(txTimeout);
             networkFlags &= ~FLAG_FAST_FRAG;
             radio.setAutoAck(0,0);
		  }
          radio.startListening();
        #endif
		uint32_t reply_time = HAL_GetTick();

		while( update() != NETWORK_ACK){
			if(HAL_GetTick() - reply_time > routeTimeout){
				  ////IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%lu: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r"),HAL_GetTick(),to_node,conversion.send_node,conversion.send_pipe); );
				ok=false;
				break;
			}
		}
    }
    if( !(networkFlags & FLAG_FAST_FRAG) ){
	   #if !defined (DUAL_HEAD_RADIO)
         // Now, continue listening
         radio.startListening();
       #endif
	}

#if defined ENABLE_NETWORK_STATS
  if(ok == true){
			++nOK;
  }else{	++nFails;
  }
#endif
  return ok;
}

/******************************************************************/

	// Provided the to_node and directTo option, it will return the resulting node and pipe
void RF24Network::logicalToPhysicalAddress(logicalToPhysicalStruct *conversionInfo){

  //Create pointers so this makes sense.. kind of
  //We take in the to_node(logical) now, at the end of the function, output the send_node(physical) address, etc.
  //back to the original memory address that held the logical information.
  uint16_t *to_node = &conversionInfo->send_node;
  uint8_t *directTo = &conversionInfo->send_pipe;
  bool *multicast = &conversionInfo->multicast;

  // Where do we send this?  By default, to our parent
  uint16_t pre_conversion_send_node = parent_node;

  // On which pipe
  uint8_t pre_conversion_send_pipe = parent_pipe;

 if(*directTo > TX_ROUTED ){
	pre_conversion_send_node = *to_node;
	*multicast = 1;
	//if(*directTo == USER_TX_MULTICAST || *directTo == USER_TX_TO_PHYSICAL_ADDRESS){
		pre_conversion_send_pipe=0;
	//}
  }
  // If the node is a direct child,
  else
  if ( is_direct_child(*to_node) )
  {
    // Send directly
    pre_conversion_send_node = *to_node;
    // To its listening pipe
    pre_conversion_send_pipe = 5;
  }
  // If the node is a child of a child
  // talk on our child's listening pipe,
  // and let the direct child relay it.
  else if ( is_descendant(*to_node) )
  {
    pre_conversion_send_node = direct_child_route_to(*to_node);
    pre_conversion_send_pipe = 5;
  }

  *to_node = pre_conversion_send_node;
  *directTo = pre_conversion_send_pipe;

}

/********************************************************/


bool RF24Network::write_to_pipe( uint16_t node, uint8_t pipe, bool multicast )
{
  bool ok = false;

  #if !defined (DUAL_HEAD_RADIO)
  // Open the correct pipe for writing.
  // First, stop listening so we can talk

  if(!(networkFlags & FLAG_FAST_FRAG)){
    radio.stopListening();
  }

  if(multicast){ radio.setAutoAck(0,0);}else{radio.setAutoAck(0,1);}

  radio.openWritingPipe(pipe_address( node, pipe ));

  ok = radio.writeFast(frame_buffer, frame_size,0);

  if(!(networkFlags & FLAG_FAST_FRAG)){
    ok = radio.txStandBy(txTimeout);
    radio.setAutoAck(0,0);
  }

#else
  radio1.openWritingPipe(pipe_address( node, pipe ));
  radio1.writeFast(frame_buffer, frame_size);
  ok = radio1.txStandBy(txTimeout,multicast);

#endif

/*  #if defined (__arm__) || defined (RF24_LINUX)
  //IF_SERIAL_DEBUG(printf_P(PSTR("%u: MAC Sent on %x %s\n\r"),HAL_GetTick(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));
  #else
  //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sent on %lx %S\n\r"),HAL_GetTick(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));
  #endif
*/
  return ok;
}

/******************************************************************/

const char* RF24NetworkHeader::toString(void) const
{
  static char buffer[45];
  //snprintf_P(buffer,sizeof(buffer),PSTR("id %04x from 0%o to 0%o type %c"),id,from_node,to_node,type);
  //sprintf_P(buffer,PSTR("id %u from 0%o to 0%o type %d"),id,from_node,to_node,type);
  return buffer;
}

/******************************************************************/

bool RF24Network::is_direct_child( uint16_t node )
{
  bool result = false;

  // A direct child of ours has the same low numbers as us, and only
  // one higher number.
  //
  // e.g. node 0234 is a direct child of 034, and node 01234 is a
  // descendant but not a direct child

  // First, is it even a descendant?
  if ( is_descendant(node) )
  {
    // Does it only have ONE more level than us?
    uint16_t child_node_mask = ( ~ node_mask ) << 3;
    result = ( node & child_node_mask ) == 0 ;
  }
  return result;
}

/******************************************************************/

bool RF24Network::is_descendant( uint16_t node )
{
  return ( node & node_mask ) == node_address;
}

/******************************************************************/

void RF24Network::setup_address(void)
{
  // First, establish the node_mask
  uint16_t node_mask_check = 0xFFFF;
  #if defined (RF24NetworkMulticast)
  uint8_t count = 0;
  #endif

  while ( node_address & node_mask_check ){
    node_mask_check <<= 3;
  #if defined (RF24NetworkMulticast)
	  count++;
  }
  multicast_level = count;
  #else
  }
  #endif

  node_mask = ~ node_mask_check;

  // parent mask is the next level down
  uint16_t parent_mask = node_mask >> 3;

  // parent node is the part IN the mask
  parent_node = node_address & parent_mask;

  // parent pipe is the part OUT of the mask
  uint16_t i = node_address;
  uint16_t m = parent_mask;
  while (m)
  {
    i >>= 3;
    m >>= 3;
  }
  parent_pipe = i;

  ////IF_SERIAL_DEBUG_MINIMAL( printf_P(PSTR("setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r"),node_address,node_mask,parent_node,parent_pipe););
//  ////IF_SERIAL_DEBUG_MINIMAL(Serial.print(F("setup_address node=")));
//  ////IF_SERIAL_DEBUG_MINIMAL(Serial.print(node_address,OCT));
//  ////IF_SERIAL_DEBUG_MINIMAL(Serial.print(F(" parent=")));
//  ////IF_SERIAL_DEBUG_MINIMAL(Serial.println(parent_node,OCT));

}

/******************************************************************/
uint16_t RF24Network::addressOfPipe( uint16_t node, uint8_t pipeNo )
{
		//Say this node is 013 (1011), mask is 077 or (00111111)
		//Say we want to use pipe 3 (11)
        //6 bits in node mask, so shift pipeNo 6 times left and | into address
	uint16_t m = node_mask >> 3;
	uint8_t i=0;

	while (m){ 	   //While there are bits left in the node mask
		m>>=1;     //Shift to the right
		i++;       //Count the # of increments
	}
    return node | (pipeNo << i);
}

/******************************************************************/

uint16_t RF24Network::direct_child_route_to( uint16_t node )
{
  // Presumes that this is in fact a child!!
  uint16_t child_mask = ( node_mask << 3 ) | 0x07;
  return node & child_mask;

}

/******************************************************************/

bool RF24Network::is_valid_address( uint16_t node )
{
  bool result = true;
  if(node == 0100 || node == 010){ return result; }
  uint8_t count = 0;
  #if defined (SERIAL_DEBUG_MINIMAL)
  uint16_t origNode = node;
  #endif
  while(node)
  {
    uint8_t digit = node & 0x07;
    if (digit < 1 || digit > 5)
    {
      result = false;
      ////IF_SERIAL_DEBUG_MINIMAL(printf_P(PSTR("*** WARNING *** Invalid address 0%o\n\r"),origNode););
      break;
    }
    node >>= 3;
    count++;
  }

  if(count > 4){
    ////IF_SERIAL_DEBUG_MINIMAL(printf_P(PSTR("*** WARNING *** Invalid address 0%o\n\r"),origNode););
    return false;
  }
  return result;
}

/******************************************************************/
#if defined (RF24NetworkMulticast)
void RF24Network::multicastLevel(uint8_t level){
  multicast_level = level;
  radio.stopListening();
  radio.openReadingPipe(0,pipe_address(levelToAddress(level),0));
  radio.startListening();
}

/******************************************************************/

uint16_t RF24Network::levelToAddress(uint8_t level){

	uint16_t levelAddr = 1;
	if(level){
		levelAddr = levelAddr << ((level-1) * 3);
	}else{
		return 0;
	}
	return levelAddr;
}
#endif
/******************************************************************/

uint64_t RF24Network::pipe_address( uint16_t node, uint8_t pipe )
{

  static uint8_t address_translation[] = { 0xc3,0x3c,0x33,0xce,0x3e,0xe3,0xec };
  uint64_t result = 0xCCCCCCCCCCLL;
  uint8_t* out = reinterpret_cast<uint8_t*>(&result);

  // Translate the address to use our optimally chosen radio address bytes
	uint8_t count = 1; uint16_t dec = node;

	while(dec){
	  #if defined (RF24NetworkMulticast)
	  if(pipe != 0 || !node)
      #endif
		out[count]=address_translation[(dec % 8)];		// Convert our decimal values to octal, translate them to address bytes, and set our address

	  dec /= 8;
	  count++;
	}

	#if defined (RF24NetworkMulticast)
	if(pipe != 0 || !node)
	#endif
	  out[0] = address_translation[pipe];
	#if defined (RF24NetworkMulticast)
	else
	  out[1] = address_translation[count-1];
	#endif


  //IF_SERIAL_DEBUG(uint32_t* top = reinterpret_cast<uint32_t*>(out+1);printf_P(PSTR("%lu: NET Pipe %i on node 0%o has address %lx%x\n\r"),HAL_GetTick(),pipe,node,*top,*out));


  return result;
}

void RF24Network::DebugUART_Init(UART_HandleTypeDef nrf24Uart)
{
	memcpy(&nrf24_huart, &nrf24Uart, sizeof(nrf24Uart));
}


/************************ Sleep Mode ******************************************/


#if defined ENABLE_SLEEP_MODE

#if !defined(__arm__) && !defined(__ARDUINO_X86__)

void wakeUp(){
  wasInterrupted=true;
  sleep_cycles_remaining = 0;
}

ISR(WDT_vect){
  --sleep_cycles_remaining;
}


bool RF24Network::sleepNode( unsigned int cycles, int interruptPin, uint8_t INTERRUPT_MODE){
  sleep_cycles_remaining = cycles;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  if(interruptPin != 255){
    wasInterrupted = false; //Reset Flag
	//LOW,CHANGE, FALLING, RISING correspond with the values 0,1,2,3 respectively
	attachInterrupt(interruptPin,wakeUp, INTERRUPT_MODE);
  	//if(INTERRUPT_MODE==0) attachInterrupt(interruptPin,wakeUp, LOW);
	//if(INTERRUPT_MODE==1) attachInterrupt(interruptPin,wakeUp, RISING);
	//if(INTERRUPT_MODE==2) attachInterrupt(interruptPin,wakeUp, FALLING);
	//if(INTERRUPT_MODE==3) attachInterrupt(interruptPin,wakeUp, CHANGE);
  }

  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  WDTCR |= _BV(WDIE);
  #else
  WDTCSR |= _BV(WDIE);
  #endif

  while(sleep_cycles_remaining){
    sleep_mode();                        // System sleeps here
  }                                     // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out
  detachInterrupt(interruptPin);

  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	WDTCR &= ~_BV(WDIE);
  #else
	WDTCSR &= ~_BV(WDIE);
  #endif
  return !wasInterrupted;
}

void RF24Network::setup_watchdog(uint8_t prescalar){

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag

  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  WDTCR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
  #else
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
  #endif
}


#endif // not ATTiny
#endif // Enable sleep mode