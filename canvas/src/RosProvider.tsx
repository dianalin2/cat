import { Ros } from 'roslib';
import { createContext, useState, useEffect } from 'react';

export const RosContext = createContext<{ ros: Ros | null; isConnected: boolean }>({ ros: null, isConnected: false });

export const RosProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [ros, setRos] = useState<Ros | null>(null);
    const [isConnected, setIsConnected] = useState(false);

    useEffect(() => {
        const rosInstance = new Ros({
            url: 'ws://localhost:9090'
        });

        rosInstance.on('connection', () => {
            console.log('Connected to ROS');
            setIsConnected(true);
        });

        rosInstance.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
            setIsConnected(false);
        });

        rosInstance.on('close', () => {
            console.log('Connection to ROS closed');
            setIsConnected(false);
        });

        setRos(rosInstance);

        return () => {
            rosInstance.close();
        };
    }, []);

    return (
        <RosContext.Provider value={{ ros, isConnected }}>
            {children}
        </RosContext.Provider>
    );
}

