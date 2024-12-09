#pragma once
#include "SerialInterface.h" //Copy from ros2 package!

///<summary>
/// Change this define if you need more callbacks!
///</summary>
#define MMI_PROTOCOL_NUM_CALLBACKS 10
#define MMI_PROTOCOL_NUM_PUBLISHERS 10

MMI_NAMESPACE_BEGIN(mmi)

//Forward declare MMIProtocol for ProtocolCallback
class MMIProtocol; 

///<summary>
/// Method callback definition
/// When handling a message, this method should return the success state
/// Return true if the message was handled correctly, otherwise false
///</summary>
/// <param name="message">The whole message received, handle additional parameters if needed</param>
/// <param name="protocol">A reference to the MMIProtocol which received the message</param>
/// <returns>True if the message was handled correctly, otherwise false</returns>
typedef mmi::Bool(*ProtocolCallback)(MMI_CONST String& message, mmi::MMIProtocol& protocol);

///<summary>
/// Publisher method definition.
/// One can add additional messahe publishers through their register/deregister methods!
///</summary>
/// <param name="protocol">A reference to the MMIProtocol</param>
typedef void (*ProtocolPublisher)(mmi::MMIProtocol& protocol);

/// <summary>
/// The MMI Prtocol class.
/// Responsible for handling connection and protocol specific
/// tasks by using the arduinos hardware serial interface.
/// </summary>
class MMIProtocol {
private:
	/// <summary>
	/// Initialization state of the current connection
	/// </summary>
	mmi::Bool m_Initialized = false;

	/// <summary>
	/// Internal message to listen for an initialization message
	/// </summary>
	String m_Message;

  /// <summary>
	/// Internal message to listen for an protocol message
	/// </summary>
  String m_ProtocolMessage;

  /// <summary>
	/// Message callback entry.
  /// This is used to store the callback functions used to handle the message
	/// </summary>
  struct CallbackEntry 
  {
    /// The message this callback entry handles
    /// We specifically use mmi::String in order to stay as flexible as possible
    mmi::String Message;

    /// The callback method
    mmi::ProtocolCallback Callback;

    CallbackEntry() : Message(""), Callback(nullptr) 
    {}
  };

   /// <summary>
	/// Message publisher entry.
  /// This is used to store the publisher functions used to send messages
	/// </summary>
  struct PublisherEntry
  {
    /// The publisher
    mmi::ProtocolPublisher Publish;

    PublisherEntry() : Publish(nullptr) 
    {}
  };

  /// The fixed size publishers and callbacks
  /// This could defenitely be optimized :D

  PublisherEntry m_Publishers[MMI_PROTOCOL_NUM_PUBLISHERS];
  CallbackEntry m_Callbacks[MMI_PROTOCOL_NUM_CALLBACKS];

  /// Last publish time
  mmi::UInt64 m_LastPublishTime = 0;

  /// Publishing interval
  /// Note: This is "only" an UInt32 instead of UInt64
  mmi::UInt32 m_PublishInterval = 1000;

public:

	/// <summary>
	/// Constructor.
	/// Initializes a new MMIProcotol object
	/// </summary>
	MMIProtocol();

	/// <summary>
	/// This method is used to register callback methods which handle the message type received.
  /// For better efficiency, internally, there is a maximum number of available callback slots
  /// defined by MMI_PROTOCOL_NUM_CALLBACKS. After including MMIProtocol.h you can modify this 
  /// define based on how many callbacks you have, not the more callbacks, the slower it will run!
  /// If registration fails, this method will return false, otherwise it will return true!
	/// </summary>
  /// <param name="messageType">The message to register this handler for</param>
  /// <param name="clb">The callback method which should be called when the given message is received</param>
  mmi::Bool RegisterMessageHandler(MMI_CONST mmi::String& messageType, mmi::ProtocolCallback clb);

  /// <summary>
	/// Registers a message publisher.
  /// Each publisher is called based on a global publisher timer. This timer is per default set to once per second.
  /// Additionally, the pushers function in the same way as the Handlers such that if you need more than the default number
  /// of publishers (10), you have to change the MMI_PROTOCOL_NUM_PUBLISHERS define at the top of MMIProtocol.h!
  ///
  /// It should be noted that all publishers run in sereal and not in their own threads!
  /// They are blocking!
	/// </summary>
  /// <param name="publisher">The publisher method to be called</param>
  mmi::Bool RegisterMessagePublisher(mmi::ProtocolPublisher publisher);

  /// <summary>
	/// Removes an already registered message publisher.
	/// </summary>
  /// <param name="publisher">The publisher method to be removed</param>
  mmi::Bool UnregisterMessagePublisher(mmi::ProtocolPublisher publisher);

  /// <summary>
	/// Sets the global publisher interval.
	/// All registered publisher methods are only called once every interval.
  /// Interval values are in milli seconds. The interval must be > 0!
	/// </summary>
	/// <param name="interval">The interval time in milliseconds</param>
  void SetPublishInterval(mmi::UInt64 interval);

	/// <summary>
	/// Begins a MMIProtocol connection.
	/// It starts the serial connection with the given baud
	/// </summary>
	/// <param name="baud">The baud to use for the serial connection</param>
	void Begin(const mmi::SerialBaud& baud);

	/// <summary>
	/// Writes a message to the serial connection.
	/// Typically this is used to write responses such as mmi::SerialProtocol::Ok
	/// to indicate the status of something.
	/// It is up to the receiver to handle the message. It does not require to send a response!
	/// </summary>
	/// <param name="protocol">The protocol or message to send</param>
	void Write(const String& protocol);

	/// <summary>
	/// Begins a new message.
	/// This method is used to receive a new message in a convenient way.
	/// If this method returns true, that means that the protocol received
	/// <c>mmi::SerialProtocol::MessageEnd</c>.. It is not guaranteed that the message
	/// which was received corresponds to any protocol message. It is up to the 
	/// user to perform the necessary checks. 
	/// The message should ideally be defined as global variable or in a similar scope.
	/// 
	///
	/// A typical use case looks like the following:
	/// <code>
	/// if (Protocol.BeginMessage(message)) 
	/// {
	///     if (Protocol.IsProtocol(message, mmi::SerialProtocol::XXXX)) {}
	///     else if (Protocol.IsProtocol(message, mmi::SerialProtocol::XXXX)) {}
	///     // ...
	///     Protocol.EndMessage(message);
	/// }
	/// </code>
	/// </summary>
	/// <param name="message">The received message</param>
	/// <returns>True if <c>mmi::SerialProtocol::MessageEnd</c> was received - indicating that a full protocol message was received, otherwise false</returns>
	mmi::Bool BeginMessage(String& message);

	/// <summary>
	/// Ends and resets the message
	/// </summary>
	/// <param name="message">The message to reset</param>
	void EndMessage(String& message);

	/// <summary>
	/// Ends and closes the serial connection and resets the internal state of this protocol instance
	/// </summary>
	void End();

	/// <summary>
	/// Tries to establish a connection.
	/// Once a connection over the serial interface has been established internal states will be changed
	/// which are reflected in ConnectionEstablished().
	/// 
	/// /// <code>
	/// //check if we established a valid connection through protocol
	/// if (Protocol.ConnectionEstablished()) 
	/// {
	///	  //We wait until we have a message
	///	  if (Protocol.BeginMessage(message)) 
	///   {
	///	  	//...
	///	  	Protocol.EndMessage(message);
	///	  }
	/// }
	/// //If we don't have an established connection -> wait until we do
	/// else
	/// {
	///	  Protocol.EstablishConnection();
	/// }
	/// </code>
	/// </summary>
	void EstablishConnection();

	/// <summary>
	/// Indicates if a valid connection has been established or not.
	/// 
	/// /// <code>
	/// //check if we established a valid connection through protocol
	/// if (Protocol.ConnectionEstablished()) 
	/// {
	///	  //We wait until we have a message
	///	  if (Protocol.BeginMessage(message)) 
	///   {
	///	  	//...
	///	  	Protocol.EndMessage(message);
	///	  }
	/// }
	/// //If we don't have an established connection -> wait until we do
	/// else
	/// {
	///	  Protocol.EstablishConnection();
	/// }
	/// </code>
	/// </summary>
	/// /// <returns>True if we have a valid connection, otherwise false</returns>
	mmi::Bool ConnectionEstablished();

	/// <summary>
	/// Checks if a given message indicates that it is valid protocol message
	/// </summary>
	/// <param name="message">The message to verify</param>
	/// <param name="protocol">The protocol to check</param>
	/// <returns>True if the message is (a sub-type or) corresponds to the given protocol, otherwise false</returns>
	mmi::Bool IsProtocol(const String& message, const String& protocol);

  /// <summary>
	/// Runs the internal message handler.
  /// This method must be called in the void loop() method!
	/// </summary>
  mmi::Bool Run();

};

MMI_NAMESPACE_END(mmi)