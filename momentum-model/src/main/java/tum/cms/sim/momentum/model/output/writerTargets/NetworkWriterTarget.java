/*******************************************************************************
 * Welcome to the pedestrian simulation framework MomenTUM. 
 * This file belongs to the MomenTUM version 2.0.2.
 * 
 * This software was developed under the lead of Dr. Peter M. Kielar at the
 * Chair of Computational Modeling and Simulation at the Technical University Munich.
 * 
 * All rights reserved. Copyright (C) 2017.
 * 
 * Contact: peter.kielar@tum.de, https://www.cms.bgu.tum.de/en/
 * 
 * Permission is hereby granted, free of charge, to use and/or copy this software
 * for non-commercial research and education purposes if the authors of this
 * software and their research papers are properly cited.
 * For citation information visit:
 * https://www.cms.bgu.tum.de/en/31-forschung/projekte/456-momentum
 * 
 * However, further rights are not granted.
 * If you need another license or specific rights, contact us!
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

package tum.cms.sim.momentum.model.output.writerTargets;

import javax.jms.Connection;
import javax.jms.Destination;
import javax.jms.JMSException;
import javax.jms.MessageProducer;
import javax.jms.Session;
import javax.jms.TextMessage;

import org.apache.activemq.ActiveMQConnectionFactory;

import tum.cms.sim.momentum.data.output.WriterData;
import tum.cms.sim.momentum.infrastructure.execute.SimulationState;

/**
 * This class uses activemq to send WriterData into a network.
 *  
 * @author Peter M. Kielar, Benedikt Schwab
 *
 */
public class NetworkWriterTarget extends WriterTarget {
	
	private final static String hostAddressString = "hostAddress";
	private final static String networkTopicString = "networkTopic";
	
	private Connection connection = null;
	private Session session = null;
	private Destination destination = null;
	private MessageProducer messageProducer = null;
	
	@Override
	public void initialize(SimulationState simulationState) {
		
		ActiveMQConnectionFactory connectionFactory = new ActiveMQConnectionFactory(
				this.properties.getStringProperty(hostAddressString));
		
        try {
        	
            connection = connectionFactory.createConnection();
			connection.start();
			session = connection.createSession(false, Session.AUTO_ACKNOWLEDGE);
			destination = session.createTopic(this.properties.getStringProperty(networkTopicString));
	        messageProducer = session.createProducer(destination);
		} 
        catch (JMSException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void writeData(WriterData writerData) {
		String writerDataString = writerData.getData();

		try {
        	TextMessage message = session.createTextMessage(writerDataString);
        	messageProducer.send(message);
        }
		 catch (Exception e) {
	        e.printStackTrace();
	    }
	}

	@Override
	public void close() {
		
		try {
			messageProducer.close();
			session.close();
			connection.close();
		}
		catch (JMSException e) {
			e.printStackTrace();
		}

	}

}
