#pragma once
#include "SerialInterface.h"

MMI_NAMESPACE_BEGIN(mmi)

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

public:

	/// <summary>
	/// Constructor.
	/// Initializes a new MMIProcotol object
	/// </summary>
	MMIProtocol();

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

};

MMI_NAMESPACE_END(mmi)