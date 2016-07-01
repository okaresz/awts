#ifndef SETTINGSMANAGER_H
#define SETTINGSMANAGER_H

#include <QSettings>

/// Application settings manager (singleton).
class SettingsManager : public QSettings
{
	Q_OBJECT

public:
	SettingsManager( const QString configFilePath, QObject *parent );
	SettingsManager( QObject *parent );
	~SettingsManager();

	static bool checkLocalConfigFile(QString &foundFile);
	static SettingsManager* instance(QObject *parent = 0);

private:
	void initDefaults();

	static SettingsManager *instancePtr;
};

#endif // SETTINGSMANAGER_H
