package com.comino.mavlquac.ftpserver;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.apache.ftpserver.ConnectionConfigFactory;
import org.apache.ftpserver.FtpServer;
import org.apache.ftpserver.FtpServerFactory;
import org.apache.ftpserver.ftplet.Authority;
import org.apache.ftpserver.ftplet.FtpException;
import org.apache.ftpserver.ftplet.UserManager;
import org.apache.ftpserver.listener.ListenerFactory;
import org.apache.ftpserver.usermanager.PropertiesUserManagerFactory;
import org.apache.ftpserver.usermanager.SaltedPasswordEncryptor;
import org.apache.ftpserver.usermanager.impl.BaseUser;
import org.apache.ftpserver.usermanager.impl.WritePermission;
import org.slf4j.LoggerFactory;

import com.comino.mavcom.config.MSPConfig;

public class MAVFtpServerFactory {

	public static FtpServer createAndStart() throws FtpException {

		MSPConfig config = MSPConfig.getInstance();
		if(config == null)
			throw new FtpException("Config not available");
		
		boolean enabled = config.getBoolProperty("ftp_enabled", "true");
		if(!enabled)
			throw new FtpException("FTP server not enabled");
		
		String user      = config.getProperty("ftp_user", "lquac");
		String pwd       = config.getProperty("ftp_pwd",  "lquac");
		int    port      = config.getIntProperty("ftp_port", "21");
		String homeDir   = config.getProperty("ftp_home",MSPConfig.getInstance().getBasePath());
		
		System.out.println("FTP home directory is: "+homeDir);
		
		File fhd = new File( homeDir );
		if( !fhd.exists() ) fhd.mkdirs();
		
		ListenerFactory listenerFactory = new ListenerFactory();
		listenerFactory.setPort( port );

		PropertiesUserManagerFactory userManagerFactory = new PropertiesUserManagerFactory();
		userManagerFactory.setPasswordEncryptor( new SaltedPasswordEncryptor() );

		UserManager userManager = userManagerFactory.createUserManager();
		BaseUser userWr = new BaseUser();
		userWr.setName( user );
		userWr.setPassword( pwd );
		userWr.setHomeDirectory( homeDir );
		userWr.setMaxIdleTime( 10 );
		List<Authority> authorities = new ArrayList<Authority>();
		authorities.add( new WritePermission() );
		userWr.setAuthorities( authorities );
		userManager.save( userWr );

		FtpServerFactory serverFactory = new FtpServerFactory();
		serverFactory.addListener( "default", listenerFactory.createListener() );
		serverFactory.setUserManager( userManager );
		ConnectionConfigFactory ccf = new ConnectionConfigFactory();
		ccf.setMaxLogins( 1 );
		serverFactory.setConnectionConfig( ccf.createConnectionConfig() );

		FtpServer server = serverFactory.createServer();
		server.start();
		
		System.out.println("FTPServer started on port "+port);

		return server;
	}



}
