#include "SettingsManager.h"
#include <QFileInfo>
#include <QCoreApplication>
#include <QDebug>

SettingsManager *SettingsManager::instancePtr = 0;

SettingsManager::SettingsManager(const QString configFilePath, QObject *parent) : QSettings( configFilePath, QSettings::defaultFormat(), parent )
{
	initDefaults();
}

SettingsManager::SettingsManager(QObject *parent) : QSettings( parent )
{
	initDefaults();
}

SettingsManager::~SettingsManager()
{

}

bool SettingsManager::checkLocalConfigFile(QString &foundFile)
{
	QString configName = QCoreApplication::applicationName();
	QString unixExt = ".conf";
	if( QSettings::defaultFormat() == QSettings::IniFormat )
		{ unixExt = ".ini"; }
#ifdef Q_OS_WIN32
		configName += ".ini";
#else
	configName += unixExt;
#endif

	QFileInfo configInfo( configName );
	if( configInfo.exists() && configInfo.isReadable() )
	{
		foundFile = configName;
		return true;
	}
	else
	{
		if( configInfo.exists() )
			{ qWarning() << "Found config file" << configName << " in the current dir, but it's not readable or otherwise invalid"; }
		foundFile = "";
		return false;
	}
}

SettingsManager *SettingsManager::instance(QObject *parent)
{
	if( !instancePtr )
	{
		QString localConf;
		if( checkLocalConfigFile( localConf ) )
			{ instancePtr = new SettingsManager( localConf, parent ); }
		else
			{ instancePtr = new SettingsManager(parent); }
	}
	return (SettingsManager*)instancePtr;
}

void SettingsManager::initDefaults()
{
//	if( !contains("localDevice/defaultLocalDevice") )
//		{ setValue( "localDevice/defaultLocalDevice", "0" ); }
}
